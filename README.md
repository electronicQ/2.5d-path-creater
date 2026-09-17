# muti_path_creater

本工程部署在移动车辆工控机上，负责把已有的局部 ENU 点云地图转换为二维栅格地图，并根据起点、目标点生成三组全局轨迹。当前分支不负责在线建图；点云、ENU 原点和规划参数由上位机通过 TCP 发送到车端。

## 端到端流程

```text
上位机上传 PCD + ENU 原点
-> 车端原子保存到 map/pcd 和 tools/path_creater/config
-> grid_map_pcl 生成 elevation GridMap
-> filter chain 生成 traversability
-> pgm_saver 保存 PGM/YAML 栅格地图
-> path_creater 根据经纬高起点/目标点规划
-> 返回轨迹文件清单和状态
```

车端同一时间只执行一个转换或规划任务。PCD 使用分块传输、块校验、完整 SHA-256 校验和断点续传。

## 目录和关键文件

| 路径 | 作用 |
| --- | --- |
| `tools/vehicle_gateway/` | TCP 网关、任务状态机、上传存储和流程编排。 |
| `map/pcd/` | 活动点云目录，默认文件为 `scans_Global_ENU.pcd`。 |
| `map/pgm/` | 转换后的 `.pgm` 和 `.yaml` 静态地图。 |
| `tools/path_creater/config/map_enu.meta.yaml` | 活动地图的 ENU 原点经纬高。 |
| `tools/path_creater/config/config.yaml` | 规划器配置。 |
| `var/vehicle_gateway/` | 上传临时文件、任务目录、日志和 manifest。 |

## 启动车端网关

```bash
source devel/setup.bash
python3 -m tools.vehicle_gateway.main \
  --config tools/vehicle_gateway/gateway.yaml
```

正式部署：

```bash
sudo cp tools/vehicle_gateway/vehicle-gateway.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vehicle-gateway.service
```

部署前修改 `tools/vehicle_gateway/gateway.yaml` 中的监听地址、端口和令牌。生产环境建议配置防火墙，仅允许上位机 IP 访问 TCP 端口。

## TCP 操作顺序

协议帧格式为网络字节序：

```text
uint32 frame_length
uint32 json_header_length
UTF-8 JSON header
binary payload
```

主要消息为 `hello`、`map.upload.begin`、`map.upload.chunk`、`map.upload.commit`、`map.convert.start`、`plan.start`、`job.status`、`job.cancel`、`artifact.list` 和 `artifact.download`。TCP 不保证一次发送对应一次接收，客户端必须按长度读取完整帧。

### 1. 上传 PCD 和 ENU 原点

`map.upload.begin` 携带 `map_id`、`pcd_name`、`file_size`、完整 SHA-256 和 `enu_origin`。随后发送多个 `map.upload.chunk`，二进制内容放在 payload，JSON 头携带 `offset` 和可选块 SHA-256。车端响应 `next_offset`，断线后重新 begin 即可续传。

提交 `map.upload.commit` 后，车端校验完整文件和 PCD 头，并原子更新：

```text
map/pcd/scans_Global_ENU.pcd
tools/path_creater/config/map_enu.meta.yaml
```

### 2. 转换并保存 PGM 地图

发送 `map.convert.start`，携带 `map_id`、`pcd_name` 和 `pgm_name`。车端无 RViz 执行：

```bash
roslaunch grid_map_pcl grid_map_pcl_loader_node.launch pcd_filename:=scans_Global_ENU.pcd enable_rviz:=false
roslaunch grid_map_demos filters_demo_gai.launch enable_rviz:=false
python3 tools/pgm_saver/save_continuous_map.py -n <任务临时目录>/campus
```

成功后生成 `map/pgm/campus.pgm` 和 `map/pgm/campus.yaml`，服务端会检查话题、输出并回收 ROS 进程组。

### 3. 开始规划

`plan.start` 携带 `map_id`、`pgm_name`、`start`、`goal` 和可选白名单 `config_overrides`。车端执行 `python3 -m tools.path_creater.main --config tools/path_creater/config/config.yaml`。结果通过 `job.status` 和 `artifact.list` 返回路径、大小和 SHA-256：

```text
traj_distance_geo.csv    traj_distance_enu.csv
traj_smooth_geo.csv      traj_smooth_enu.csv
traj_cost_aware_geo.csv  traj_cost_aware_enu.csv
road_map_distance.xml    road_map_smooth.xml
road_map_cost_aware.xml
overlay.png              config.resolved.yaml
```

## 状态和错误

典型状态：`UPLOADING -> READY -> CONVERTING -> MAP_READY -> PLANNING -> SUCCEEDED`。失败进入 `FAILED`，常见错误码包括 `INVALID_ARGUMENT`、`CHECKSUM_MISMATCH`、`BUSY`、`FILE_NOT_FOUND`、`ROS_UNAVAILABLE`、`MAP_TOPIC_TIMEOUT`、`PROCESS_FAILED`、`OUTPUT_INVALID`、`PLANNER_FAILED`、`JOB_TIMEOUT` 和 `JOB_CANCELLED`。任务执行期间新任务返回 `BUSY`，可发送 `job.cancel`。

## 手动调试

```bash
source devel/setup.bash
roslaunch grid_map_pcl grid_map_pcl_loader_node.launch enable_rviz:=false
roslaunch grid_map_demos filters_demo_gai.launch enable_rviz:=false
python3 tools/pgm_saver/save_continuous_map.py -n campus
python3 -m tools.path_creater.main
```

## 坐标和地图语义

工程同时使用经纬高、局部 ENU 米制坐标和 PGM 栅格索引，不能混用。规划器将经纬高起点、目标点转换为 ENU 和栅格坐标，输出经纬度 CSV 与 ENU CSV。

- PGM `205`：未知区域，禁止通行；
- PGM `<= 90`：绝对障碍；
- 其他有效灰度：连续通行代价，灰度越大代价越低。

未观测区域必须保持：`elevation NaN -> traversability NaN -> OccupancyGrid -1 -> PGM 205`。

详细规划参数见 [`tools/path_creater/README.md`](tools/path_creater/README.md)，网关细节见 [`tools/vehicle_gateway/README.md`](tools/vehicle_gateway/README.md)，长期约定见 [`docs/project-context.md`](docs/project-context.md)。

上位机的安装、运行和打包说明见 [`tools/ground_station/README.md`](tools/ground_station/README.md)。

地面端对接公司使用的正式 TCP 应用层协议见 [`docs/vehicle_gateway_protocol.md`](docs/vehicle_gateway_protocol.md)，其中包含帧结构、JSON 字段、PCD 上传、地图转换、路径规划、状态反馈、结果下载和错误码。
