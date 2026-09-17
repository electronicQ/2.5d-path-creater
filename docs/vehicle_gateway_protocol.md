# 车端 TCP 应用层协议

本文档是地面端上位机与车辆工控机 `vehicle_gateway` 对接的接口规范。该协议是基于 TCP 的自定义应用层协议，不是 ROS 协议；地面端不需要安装 ROS。

## 1. 网络连接

| 项目 | 约定 |
| --- | --- |
| 传输层 | TCP，IPv4 |
| 车端角色 | TCP Server，监听车辆工控机地址 |
| 默认端口 | `19090` |
| 字节序 | 所有长度字段使用网络字节序（big-endian） |
| 编码 | JSON 使用 UTF-8；文件 payload 使用原始二进制 |
| 连接方式 | 长连接；一条连接可连续发送多个请求 |
| 认证 | 连接后必须首先发送 `hello`，使用预共享 token |

车端 IP 由现场网络配置决定，地面端应提供 IP、端口和 token 配置项。两台设备必须在同一 Wi-Fi 且允许互相访问；不能使用 `127.0.0.1` 连接车端。

## 2. 帧格式

每个方向的每条消息都封装为一个完整帧：

```text
+----------------------+ 4 bytes, uint32, big-endian
| frame_length         | 后续 body 的字节数
+----------------------+
| header_length        | 4 bytes, uint32, big-endian
+----------------------+
| JSON header          | header_length bytes, UTF-8
+----------------------+
| binary payload       | frame_length - 4 - header_length bytes
+----------------------+
```

其中：

- `frame_length` 不包含自身的 4 字节；
- `header_length` 不包含自身的 4 字节；
- 没有文件内容时 payload 长度为 0；
- 当前车端默认单帧最大长度为 `8388608` 字节（8 MiB）；
- PCD 和结果文件必须分块传输，不能试图构造一个数百 MB 的单帧。

伪代码：

```text
send_u32_be(len(json_header) + 4 + len(payload))
send_u32_be(len(json_header))
send(json_header_utf8)
send(payload)
```

地面端必须循环 `recv()` 直到读满指定长度，以处理 TCP 拆包；也必须支持一次 `recv()` 收到多个帧，以处理 TCP 粘包。

## 3. JSON 公共字段

请求通常包含：

```json
{
  "type": "message.name",
  "request_id": "req-uuid",
  "job_id": "job-uuid"
}
```

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `type` | string | 消息类型，大小写敏感 |
| `request_id` | string | 请求唯一 ID；建议 UUID，响应原样返回 |
| `job_id` | string | 任务或上传会话 ID；同一任务过程中保持不变 |
| `seq` | integer | 可选的消息序号 |

## 4. 统一响应格式

无 payload 的响应示例：

```json
{
  "type": "map.convert.start.reply",
  "request_id": "req-uuid",
  "status": "accepted",
  "job_id": "convert-uuid",
  "state": "CONVERTING",
  "message": "starting ROS map conversion"
}
```

状态字段约定：

| `status` | 含义 |
| --- | --- |
| `succeeded` | 当前请求或任务成功完成 |
| `accepted` | 请求已接受，后台任务开始执行 |
| `progress` | 请求/任务仍在执行 |
| `failed` | 请求或任务失败 |

失败响应至少包含：

```json
{
  "type": "error",
  "request_id": "req-uuid",
  "status": "failed",
  "error_code": "INVALID_ARGUMENT",
  "message": "human readable detail"
}
```

## 5. 消息清单

| 请求类型 | payload | 作用 |
| --- | --- | --- |
| `hello` | 无 | 认证和协议版本协商 |
| `map.upload.begin` | 无 | 创建或恢复 PCD 上传会话 |
| `map.upload.chunk` | PCD 二进制 | 上传一个 PCD 分块 |
| `map.upload.commit` | 无 | 校验并提交完整 PCD |
| `map.convert.start` | 无 | 启动 PCD 到 PGM 转换 |
| `plan.start` | 无 | 启动路径规划 |
| `job.status` | 无 | 查询任务当前状态 |
| `job.cancel` | 无 | 取消当前任务 |
| `artifact.list` | 无 | 查询任务输出文件 |
| `artifact.download` | 无 | 分块下载任务输出文件 |

车端对每个请求返回 `<request type>.reply`，后台任务另外发送 `job.status` 推送。地面端不能只依赖推送；断线重连后必须使用 `job.status` 查询任务。

## 6. hello 认证

地面端连接后发送：

```json
{
  "type": "hello",
  "request_id": "req-hello-1",
  "token": "预共享令牌",
  "protocol": 1,
  "client": "company-ground-station",
  "client_version": "1.0.0"
}
```

成功响应：

```json
{
  "type": "hello.reply",
  "request_id": "req-hello-1",
  "status": "succeeded",
  "server": "vehicle_gateway",
  "protocol": 1
}
```

token 错误返回 `UNAUTHORIZED`。未认证前发送其他消息返回 `UNAUTHORIZED`。

## 7. PCD 上传

### 7.1 开始或恢复上传

请求 `map.upload.begin`：

```json
{
  "type": "map.upload.begin",
  "request_id": "req-upload-begin",
  "job_id": "upload-campus-001",
  "map_id": "campus_20260916",
  "pcd_name": "scans_Global_ENU.pcd",
  "file_size": 362729087,
  "sha256": "完整 PCD 文件的 64 位十六进制 SHA-256",
  "enu_origin": {
    "latitude": 22.39758935,
    "longitude": 113.30942461,
    "altitude": 25.8618
  }
}
```

约束：

- `map_id` 只能使用字母、数字、下划线、点和短横线；
- `pcd_name` 必须是 basename 且以 `.pcd` 结尾；
- 推荐固定使用 `scans_Global_ENU.pcd`；
- `file_size` 必须为正整数；
- `sha256` 必须是完整文件 SHA-256；
- 经纬度必须分别位于 `[-90, 90]`、`[-180, 180]`；
- `altitude` 使用米；
- ENU 原点必须与 PCD 内部局部 ENU 坐标系一致。

响应：

```json
{
  "type": "map.upload.begin.reply",
  "request_id": "req-upload-begin",
  "status": "accepted",
  "state": "UPLOADING",
  "next_offset": 0,
  "file_size": 362729087
}
```

`next_offset` 是车端已经持久化的字节数。若之前上传过同一 `map_id + sha256`，地面端从该 offset 继续。

### 7.2 上传分块

请求 `map.upload.chunk`：

```json
{
  "type": "map.upload.chunk",
  "request_id": "req-upload-chunk-1",
  "job_id": "upload-campus-001",
  "offset": 0,
  "sha256": "当前分块的 SHA-256"
}
```

该帧 payload 是从 PCD 文件 `offset` 位置开始的原始字节。建议每块 `4194304` 字节（4 MiB），最后一块可以更小。

响应：

```json
{
  "type": "map.upload.chunk.reply",
  "request_id": "req-upload-chunk-1",
  "status": "progress",
  "state": "UPLOADING",
  "next_offset": 4194304,
  "progress": 0.0116
}
```

车端只接受等于 `next_offset` 的 offset。offset 不匹配时，地面端应丢弃本次发送结果并从车端要求的 offset 继续。

### 7.3 提交上传

请求：

```json
{
  "type": "map.upload.commit",
  "request_id": "req-upload-commit",
  "job_id": "upload-campus-001"
}
```

车端会检查文件大小、完整 SHA-256 和 PCD 头部。成功后原子更新：

```text
map/pcd/<pcd_name>
tools/path_creater/config/map_enu.meta.yaml
```

响应状态为 `READY`，并返回 `pcd_path`、`meta_path`、`file_size` 和 `sha256`。

## 8. 地图转换

请求 `map.convert.start`：

```json
{
  "type": "map.convert.start",
  "request_id": "req-convert-1",
  "job_id": "convert-campus-001",
  "map_id": "campus_20260916",
  "pcd_name": "scans_Global_ENU.pcd",
  "pgm_name": "campus"
}
```

`pgm_name` 可以带或不带 `.pgm`，车端会规范化为 basename。转换任务执行顺序：

```text
启动 grid_map_pcl（enable_rviz=false）
-> 等待 /grid_map_pcl_loader_node/grid_map_from_raw_pointcloud
-> 启动 filters_demo（enable_rviz=false）
-> 等待 /grid_map_visualization/traversability_grid
-> 启动 save_continuous_map.py
-> 检查 .pgm 和 .yaml
-> 原子提升到 map/pgm/
-> 回收 ROS 进程组
```

接受响应只表示任务已启动，不代表转换已经完成。地面端轮询 `job.status`，直到 `SUCCEEDED` 或 `FAILED`。

## 9. 路径规划

请求 `plan.start`：

```json
{
  "type": "plan.start",
  "request_id": "req-plan-1",
  "job_id": "plan-campus-001",
  "map_id": "campus_20260916",
  "pgm_name": "campus",
  "start": {"latitude": 22.392983497, "longitude": 113.308575574, "altitude": 38.1928},
  "goal": {"latitude": 22.395137269, "longitude": 113.310275005, "altitude": 38.1875},
  "config_overrides": {
    "inflation_radius_m": 1.6,
    "d_safe_m": 4.0,
    "smooth_paths": true
  }
}
```

`config_overrides` 只能使用车端白名单字段。推荐第一版只使用：

```text
unknown_gray
absolute_obstacle_gray
inflation_radius_m
d_safe_m
smooth_paths
write_overlay
write_resolved_config
gray_cost.enabled
gray_cost.weight
curve_smoothing.enabled
curve_smoothing.sample_interval_m
curve_smoothing.corner_cut_fraction
curve_smoothing.max_corner_cut_m
curve_smoothing.min_corner_cut_m
curve_smoothing.safety_check_step_m
planners.distance/smooth/cost_aware 的 w_turn、w_clear、w_gray
```

车端执行 `python3 -m tools.path_creater.main`，成功后生成六个轨迹 CSV、三个局部 UTM `road_map_*.xml` 文件，以及 `overlay.png` 和 `config.resolved.yaml`。XML 文件分别为 `road_map_distance.xml`、`road_map_smooth.xml` 和 `road_map_cost_aware.xml`，每个文件只包含 `<map>` 根节点和 `<vertex>` 轨迹点，坐标原点与 ENU 原点一致。

## 10. 任务状态查询和推送

查询请求：

```json
{
  "type": "job.status",
  "request_id": "req-status-1",
  "job_id": "plan-campus-001"
}
```

地面端应处理以下状态：

```text
UPLOADING
READY
CONVERTING
MAP_READY
PLANNING
SUCCEEDED
FAILED
```

转换或规划开始后，车端可能主动发送：

```json
{
  "type": "job.status",
  "status": "progress",
  "job_id": "plan-campus-001",
  "state": "PLANNING",
  "message": "running path planner"
}
```

地面端必须按 `job_id` 关联状态，不要按 TCP 到达顺序猜测任务。

## 11. 取消任务

请求：

```json
{
  "type": "job.cancel",
  "request_id": "req-cancel-1",
  "job_id": "plan-campus-001"
}
```

车端先返回 `CANCELLING`，随后发送 `FAILED`，错误码为 `JOB_CANCELLED`。取消会终止本次任务启动的 ROS/规划子进程。

## 12. 查询和下载结果

### 12.1 查询文件清单

请求：

```json
{
  "type": "artifact.list",
  "request_id": "req-artifacts-1",
  "job_id": "plan-campus-001"
}
```

成功响应的 `artifacts` 是对象，键为文件名，值包含：

```json
{
  "traj_smooth_geo.csv": {
    "path": "var/vehicle_gateway/jobs/plan-campus-001/output/traj_smooth_geo.csv",
    "size": 123456,
    "sha256": "..."
  }
}
```

### 12.2 分块下载文件

请求 `artifact.download`：

```json
{
  "type": "artifact.download",
  "request_id": "req-download-1",
  "job_id": "plan-campus-001",
  "artifact": "traj_smooth_geo.csv",
  "offset": 0,
  "length": 4194304
}
```

响应 header：

```json
{
  "type": "artifact.download.reply",
  "request_id": "req-download-1",
  "status": "progress",
  "artifact": "traj_smooth_geo.csv",
  "offset": 0,
  "next_offset": 4194304,
  "file_size": 123456,
  "sha256": "完整文件 SHA-256"
}
```

响应 payload 是文件原始字节。最后一块的 `status` 为 `succeeded`。地面端应把 payload 写入 `<filename>.part`，断线重连后从本地 part 文件大小继续；完成后校验完整 SHA-256，再改名为正式文件。

车端只允许下载该任务 manifest 中列出的 artifact，并拒绝目录穿越文件名。

## 13. 错误码

| 错误码 | 含义 |
| --- | --- |
| `INVALID_ARGUMENT` | 请求字段缺失、类型错误或参数非法 |
| `UNAUTHORIZED` | token 错误或尚未 hello |
| `CHECKSUM_MISMATCH` | 分块或完整文件校验失败 |
| `BUSY` | 已有转换/规划任务执行 |
| `FILE_NOT_FOUND` | 地图、任务或 artifact 不存在 |
| `ROS_UNAVAILABLE` | ROS 命令或环境不可用 |
| `ROS_START_TIMEOUT` | ROS 启动超时 |
| `MAP_TOPIC_TIMEOUT` | 等待 GridMap 话题超时 |
| `PROCESS_FAILED` | 子进程异常退出 |
| `OUTPUT_INVALID` | 输出文件缺失或内容不一致 |
| `PLANNER_FAILED` | 路径规划失败 |
| `JOB_TIMEOUT` | 任务超时 |
| `JOB_CANCELLED` | 任务被取消 |

地面端显示给操作员的错误应包含 `error_code` 和 `message`，并保留 `request_id`、`job_id` 便于日志对照。

## 14. 推荐完整时序

```text
TCP connect
-> hello / hello.reply
-> map.upload.begin / begin.reply
-> map.upload.chunk / chunk.reply (重复直到 next_offset == file_size)
-> map.upload.commit / commit.reply (READY)
-> map.convert.start / start.reply (accepted)
-> job.status (轮询直到 MAP_READY 或 FAILED)
-> plan.start / start.reply (accepted)
-> job.status (轮询直到 SUCCEEDED 或 FAILED)
-> artifact.list / list.reply
-> artifact.download / download.reply + payload (每个文件分块)
-> 本地校验所有文件 SHA-256
TCP close
```

转换和规划请求必须串行发送。不要在 PCD 尚未 `READY` 时启动转换，也不要在地图尚未 `MAP_READY` 时启动规划。
