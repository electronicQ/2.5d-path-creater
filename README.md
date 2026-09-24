# muti_path_creater

本工程运行在车辆工控机上，使用点云地图生成二维 PGM 地图，并根据地面车辆客户端发送的局部 UTM 起点、目标点生成三组轨迹。

当前唯一的 TCP 服务是 `tools/ground_vehicle_tcp`，协议采用 `docs/TCP通信协议/通信协议.xls` 定义的定长二进制协议。旧的 JSON `vehicle_gateway` 和 PySide6 `ground_station` 已移除，不再使用。

## 服务流程

1. 客户端发送 32 字节规划请求。
2. 车端校验帧长度、字段和 CRC8。
3. 首次请求或地图指纹变化时，车端依次执行无 RViz 的 ROS 点云转换和 PGM 保存。
4. 车端将局部 UTM 坐标反投影为 WGS84，再由规划器转换到地图局部 ENU 坐标进行规划；请求中的起点和目标点 yaw 当前只接收、不参与规划。
5. 三组 CSV 和三个 `road_map*.xml` 都校验通过后，车端原子发布到 `output/`，再发送 8 字节成功响应。

地图缓存指纹包含 PCD SHA-256、ENU 原点和转换配置。服务重启时指纹匹配即可复用 PGM；PCD、原点或转换配置变化时自动重新转换。一次只允许执行一个转换/规划任务。

## 目录

```text
tools/ground_vehicle_tcp/
  protocol.py       定长帧、CRC8、请求/响应编解码
  server.py         asyncio TCP 服务端
  workflow.py       地图缓存、坐标转换、规划和输出发布
  pipeline.py       ROS 转换及规划子进程管理
  client.py         同机验证客户端
  config.yaml       服务配置
  ground-vehicle-tcp.service  systemd 服务文件
```

规划输出：

```text
output/traj_distance_geo.csv
output/traj_distance_enu.csv
output/traj_smooth_geo.csv
output/traj_smooth_enu.csv
output/traj_cost_aware_geo.csv
output/traj_cost_aware_enu.csv
output/road_map1.xml
output/road_map2.xml
output/road_map3.xml
```

`road_map1.xml`、`road_map2.xml`、`road_map3.xml` 分别对应代价感知、距离优先和平滑优先轨迹，坐标是以 ENU 原点为参考的局部 UTM 偏移，单位为米。

## 协议概要

请求帧总长 32 字节：

```text
magic(2) type(2) length(1)
start_x_mm(4) start_y_mm(4) start_yaw_mrad(4)
goal_x_mm(4) goal_y_mm(4) goal_yaw_mrad(4)
timeout_s(2) crc8(1)
```

响应帧总长 8 字节：

```text
magic(2) type(2) length(1) result(1) error_code(1) crc8(1)
```

所有多字节字段使用大端序。请求 magic 为 `0xFFDA`，响应 magic 为 `0xFFDB`，type 为 `0x0001`。CRC8 初值为 `0x00`、多项式为 `0x07`，计算范围不包含 CRC 自身。错误码 `0x00` 表示成功；`0x01` 目标越界，`0x02` 无法到达，`0x03` 起点不可规划，`0x04` 地图转换失败，`0x05` 规划超时，`0x06` 帧错误，`0x07` 内部错误或服务忙。

完整字段表、坐标转换规则和错误处理见 [`tools/ground_vehicle_tcp/README.md`](tools/ground_vehicle_tcp/README.md)。

## 启动服务

先修改 `tools/ground_vehicle_tcp/config.yaml` 中的项目根目录、PCD 路径、监听端口和转换超时：

```bash
source devel/setup.bash
python3 -m tools.ground_vehicle_tcp.main \
  --config tools/ground_vehicle_tcp/config.yaml
```

systemd 部署：

```bash
sudo cp tools/ground_vehicle_tcp/ground-vehicle-tcp.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ground-vehicle-tcp.service
```

## 同机验证客户端

服务启动后，在另一终端发送一条请求。坐标参数使用米，客户端会编码为毫米；yaw 使用弧度，服务端不会用于规划：

```bash
python3 -m tools.ground_vehicle_tcp.client \
  --host 127.0.0.1 --port 8079 \
  --start-x 0 --start-y 0 --goal-x 10 --goal-y 5 \
  --timeout 120 --response-timeout 2400
```

## 测试

只运行新协议服务端及客户端相关测试：

```bash
python3 -m unittest tests.test_ground_vehicle_tcp -v
python3 -m compileall -q tools/ground_vehicle_tcp
```
