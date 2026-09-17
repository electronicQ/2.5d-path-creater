# vehicle_gateway 车端网关

`vehicle_gateway` 是运行在车辆工控机上的常驻 TCP 服务，是地面端上位机与本工程之间的业务入口。地面端只发送约定的请求和文件数据；网关负责认证、参数校验、文件落盘、任务执行、进程监控和结果反馈。

网关不是路径规划算法，也不是 ROS 节点。它把远程请求编排成受控的车端工作流：

```text
地面端上位机
    │ TCP 长连接
    ▼
GatewayServer       解析帧、认证、分发请求
    ▼
JobManager          单任务互斥、任务状态和结果
    ├── UploadStore       PCD/ENU 原点上传与原子提交
    ├── ConfigStore       规划 YAML 白名单更新
    └── PipelineRunner    ROS 转换和路径规划子进程
```

正式对接时，消息字段、帧格式、错误码和交互顺序以 [`docs/vehicle_gateway_protocol.md`](../../docs/vehicle_gateway_protocol.md) 为准；TCP 帧原理可参考 [`tools/TCP_APPLICATION_PROTOCOL_GUIDE.md`](../TCP_APPLICATION_PROTOCOL_GUIDE.md)。

## 1. 文件职责

| 文件 | 作用 |
| --- | --- |
| `main.py` | 服务启动入口，读取配置并启动 asyncio TCP server。 |
| `service.py` | 网关核心：客户端连接、`hello` 认证、消息分发、任务互斥、状态推送、任务取消和 artifact 下载。 |
| `protocol.py` | 共享协议的兼容导出；实际帧编解码来自 `tools/vehicle_protocol.py`。 |
| `storage.py` | PCD 分块上传、断点续传、SHA-256 校验、PCD 头检查、ENU 元数据写入和活动文件原子替换。 |
| `validation.py` | basename、任务 ID、SHA-256、经纬高、PCD 头和文件摘要等安全校验。 |
| `configuration.py` | 读取和更新 `tools/path_creater/config/config.yaml`，只允许修改白名单字段，并生成备份。 |
| `runner.py` | 启动和监控 ROS launch、PGM 保存器及路径规划器，等待话题、检查返回码并清理进程组。 |
| `gateway.yaml` | 监听地址、端口、token、工程根目录、任务数据目录和超时配置。 |
| `vehicle-gateway.service` | systemd 服务文件，启动前加载 `devel/setup.bash`，支持开机启动和异常重启。 |
| `__init__.py` | 将目录声明为 Python 包。 |

## 2. 请求处理流程

`GatewayServer` 为每个 TCP 客户端创建 asyncio 处理协程。它通过 `read_frame()` 按长度读取完整应用层帧，解析 JSON header 中的 `type` 后分发业务，再用 `write_frame()` 返回响应。因此不依赖一次 `recv()` 是否刚好得到一条消息，也能处理 TCP 拆包和粘包。

服务使用长连接，一个连接可以连续发送多条请求。后台任务运行期间会推送 `job.status`；地面端断线重连后仍可查询任务状态。

如果配置了 `token`，客户端必须首先发送 `hello`。支持的主要消息为：

```text
hello
map.upload.begin / map.upload.chunk / map.upload.commit
map.convert.start
plan.start
job.status / job.cancel
artifact.list / artifact.download
```

未知消息、未认证消息、非法参数和越权文件访问都会返回失败响应；网关不会执行上位机传来的 shell 命令或任意路径。

## 3. PCD 和 ENU 原点上传

`UploadStore` 不会把未校验完成的文件直接放入活动地图目录：

1. `map.upload.begin` 校验 `map_id`、PCD basename、文件大小、完整 SHA-256 和 ENU 原点范围。
2. 分块写入 `var/vehicle_gateway/uploads/<map_id>-<sha256>.part`，并保存上传元数据。
3. `map.upload.chunk` 只接受当前 `next_offset`，每块可带 SHA-256；写入后 flush 并 `fsync`。
4. 断线重连时再次 begin，车端返回已保存的 `next_offset`，地面端从该位置继续。
5. `map.upload.commit` 重新计算完整 SHA-256，并检查 PCD 头部格式。
6. 校验通过后，PCD 和 ENU 元数据经临时文件、`fsync`、`os.replace()` 原子提交。

活动文件位置固定为：

```text
map/pcd/scans_Global_ENU.pcd
tools/path_creater/config/map_enu.meta.yaml
```

上传未完成、校验失败或提交异常时，`.part` 文件不会被转换流程当作活动地图读取。`map_id` 用于任务关联和记录，文件名仍只接受安全 basename。

## 4. 地图转换流程

`map.convert.start` 先确认指定 `map_id` 已成功提交且 PCD 存在，然后由 `PipelineRunner.convert()` 顺序执行：

```text
roslaunch grid_map_pcl grid_map_pcl_loader_node.launch
    │ 等待原始 GridMap 话题
    ▼
roslaunch grid_map_demos filters_demo_gai.launch
    │ 等待 /grid_map_visualization/traversability_grid
    ▼
python3 tools/pgm_saver/save_continuous_map.py -n <临时输出基名>
```

网关使用参数列表启动受控子进程，而不是把远程字符串交给 shell。转换期间检查 ROS 话题、进程返回码、`.pgm` 和 `.yaml` 是否生成，以及 YAML 是否正确引用 PGM。临时结果通过检查后才提升到：

```text
map/pgm/<pgm_name>.pgm
map/pgm/<pgm_name>.yaml
```

结束、超时、取消或失败时，会终止本次任务启动的 ROS 进程组并关闭日志。服务默认传入 `enable_rviz:=false`；车端 launch 文件应支持该参数，或提供无 RViz 版本。

## 5. 路径规划流程

`plan.start` 会校验 PGM/YAML、地图 ID、起点和目标点，然后只应用 `configuration.py` 中允许且在范围内的规划参数。原配置先备份为 `.previous`，本次任务使用 `var/vehicle_gateway/jobs/<job_id>/output/` 作为独立输出目录。

网关随后执行：

```bash
python3 -m tools.path_creater.main \
  --config tools/path_creater/config/config.yaml
```

规划成功必须生成以下六个 CSV：

```text
traj_distance_geo.csv       traj_distance_enu.csv
traj_smooth_geo.csv         traj_smooth_enu.csv
traj_cost_aware_geo.csv     traj_cost_aware_enu.csv
```

如果生成，还会记录 `road_map_distance.xml`、`road_map_smooth.xml`、`road_map_cost_aware.xml`、`overlay.png` 和 `config.resolved.yaml`。最终创建 `manifest.json`，记录结果文件相对路径、大小和 SHA-256。地面端先调用 `artifact.list`，再通过 `artifact.download` 分块下载；网关只允许下载 manifest 中且位于任务目录内的文件。

## 6. 任务互斥和状态

车端同时只执行一个转换或规划任务。任务运行期间，其他转换/规划请求立即返回 `BUSY`，不会排队或覆盖当前任务。典型状态为：

```text
IDLE -> UPLOADING -> READY -> CONVERTING -> MAP_READY -> PLANNING -> SUCCEEDED
```

异常统一进入 `FAILED`。常见错误码包括 `INVALID_ARGUMENT`、`CHECKSUM_MISMATCH`、`BUSY`、`FILE_NOT_FOUND`、`ROS_UNAVAILABLE`、`MAP_TOPIC_TIMEOUT`、`PROCESS_FAILED`、`OUTPUT_INVALID`、`PLANNER_FAILED` 和 `JOB_TIMEOUT`。

地面端不应只依赖异步状态推送；收到 `succeeded` 后再继续下一步，重连后主动调用 `job.status` 查询。

## 7. 任务目录和日志

```text
var/vehicle_gateway/jobs/<job_id>/
├── request.json       # 原始请求
├── job.log            # ROS、PGM 保存器或规划器输出
├── manifest.json      # 成功后的结果清单
└── output/            # 规划输出文件
```

PCD 断点上传临时文件位于 `var/vehicle_gateway/uploads/`；活动地图信息保存在 `var/vehicle_gateway/active_map.json`。这些文件用于任务恢复、问题审计和结果下载。

## 8. 配置和部署

编辑 [`gateway.yaml`](gateway.yaml)：

```yaml
host: 0.0.0.0
port: 19090
token: change-me
max_frame_size: 8388608
project_root: ../..
data_root: var/vehicle_gateway
job_timeout_s: 1800
```

正式部署至少修改 `token`，并在防火墙中只允许指定地面端 IP 访问 `19090`。两台设备在同一 Wi-Fi 时，地面端连接车端工控机的局域网 IP，不能使用 `127.0.0.1`。

手动启动：

```bash
cd /home/cclg/muti_path_creater
source devel/setup.bash
python3 -m tools.vehicle_gateway.main \
  --config tools/vehicle_gateway/gateway.yaml
```

systemd 部署：

```bash
sudo cp tools/vehicle_gateway/vehicle-gateway.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vehicle-gateway.service
```

启用 `systemctl enable` 后服务会加入开机启动；`Restart=on-failure` 会在异常退出时自动重启。车端仍必须具备可用的 ROS 环境和 ROS master。查看日志：

```bash
sudo systemctl status vehicle-gateway.service
sudo journalctl -u vehicle-gateway.service -f
```

## 9. 依赖和边界

网关需要 Python 的 `PyYAML`，并依赖本工程已构建的 ROS 环境。它不会安装 ROS，也不会替代 `roscore`；它只在收到转换请求时启动和监控相关 launch/脚本。

上位机不能发送 shell 命令、任意绝对路径或 `../` 路径，不能在上传未 commit 时要求转换，也不能直接修改车端配置或下载 manifest 之外的文件。

概括来说，`vehicle_gateway` 的作用是：

```text
不可信的远程请求
    -> 认证和参数校验
    -> 单任务、可恢复、可审计地执行车端流程
    -> 返回状态、错误和结果文件摘要
```
