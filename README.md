# muti_path_creater

本工程运行在车辆工控机上，使用点云地图生成二维 PGM 地图，并根据地面车辆客户端发送的局部 UTM 起点、目标点生成三组轨迹。

当前唯一的 TCP 服务是 `tools/ground_vehicle_tcp`，协议采用 `docs/TCP通信协议/通信协议.xls` 定义的定长二进制协议。旧的 JSON `vehicle_gateway` 和 PySide6 `ground_station` 已移除，不再使用。

## Ubuntu 18.04 适配分支与部署

本分支 `kunpengfinal_18.04` 增加 **Ubuntu 18.04 + ROS Melodic** 原生部署，同时保留 Ubuntu 20.04 + Noetic 的部署入口。部署脚本根据操作系统自动选择，不能混装两种 ROS 发行版。

```bash
git clone --branch kunpengfinal_18.04 --single-branch git@github.com:electronicQ/2.5d-path-creater.git
cd 2.5d-path-creater
# 以普通用户执行；安装包时脚本自行调用 sudo
bash scripts/deploy_vehicle.sh --setup-ros-repo --jobs 2
```

已有有效 ROS 软件源时可以省略 `--setup-ros-repo`。18.04 上脚本会安装 Melodic 和 C++/ROS 依赖，使用 rosdep 检查所有包，编译项目内 Python 3.8.20、建立独立规划环境并安装锁定版本的 Python 依赖，再以系统 Python 2.7 执行 catkin 编译。不会覆盖系统 Python，不需要将整套 Melodic 迁移到 Python 3。

```text
ROS Melodic / 系统 Python 2.7：ROS 工具、消息生成、地图保存
项目 .runtime/planner / Python 3.8：TCP 服务、坐标转换、轨迹规划
两者通过 PGM/YAML 文件衔接；运行脚本隔离 Python 模块搜索路径
```

初次部署需要联网、sudo 权限和足够的磁盘空间；Python 源码编译与 PCL 编译需要一定时间。编译并发默认 2，内存紧张可改成 1。部署日志保存在 `log/deploy/`。

```bash
bash scripts/deploy_vehicle.sh --jobs 1
# 也可分开安装和编译；build-only 不安装依赖、不访问网络
bash scripts/deploy_vehicle.sh --deps-only
bash scripts/deploy_vehicle.sh --build-only --jobs 2
```

准备 PCD 地图和原点元数据后，使用统一入口启动（18.04 不要直接运行系统 python3）：

```bash
bash scripts/run_vehicle.sh
# 自定义服务配置
bash scripts/run_vehicle.sh --config tools/ground_vehicle_tcp/config.yaml
```

18.04 上运行仓库客户端或规划器命令时，使用 `.runtime/planner/bin/python` 替换文档示例中的 `python3`，并通过 `env -u PYTHONPATH -u PYTHONHOME` 隔离 ROS Python 2 路径。例如：

```bash
env -u PYTHONPATH -u PYTHONHOME .runtime/planner/bin/python -m tools.ground_vehicle_tcp.client \
  --host 127.0.0.1 --port 8079 \
  --start-x 0 --start-y 0 --goal-x 10 --goal-y 5 \
  --timeout 120 --response-timeout 2400
```

坐标为示例，必须替换为实际地图内的可通行起终点。

详见 [Ubuntu 18.04 部署与排障](docs/deployment-ubuntu18.04.md) 和 [TCP 对接手册](tools/ground_vehicle_tcp/README.md)。20.04 上仍使用 Noetic；22.04/24.04 不属于此原生部署入口支持范围。

## 部署分工与功能边界

- **无人车工控机（服务端）**：运行本项目，监听 TCP `8079`，读取车端已有点云地图，执行地图转换和全局轨迹规划，将结果文件保存到车端。
- **地面控制平台（客户端）**：主动连接无人车 IP，按定长二进制协议发送起点、终点等参数，等待最终状态响应。仓库中的 `client.py` 可用于地面端联调，也可作为自研控制平台的参考。
- **地图传输**：当前 TCP 协议没有 PCD 上传功能。需要提前使用 SCP、SFTP、U 盘等方式将 `.pcd` 放到车端，并配置与该点云对应的原点经纬高。这是部署准备步骤，不是规划请求的一部分。
- **结果传输**：当前 TCP 只回传成功/失败和错误码，不回传轨迹点、CSV、XML、任务 ID 或进度。地面平台如需结果文件，需要另行通过 SFTP 等渠道读取。
- **车辆控制**：本项目这条链路负责生成全局轨迹；地面端收到成功响应不代表车辆已开始行驶或到达终点。实际循迹需要另接车辆控制模块。

```text
地面控制平台                       无人车工控机
提前复制 PCD/原点元数据 ──────────> 本地地图文件
TCP 客户端 ──32 字节规划请求─────> TCP 服务端 :8079
                                  校验 → 地图转换/缓存 → 三条轨迹规划
                                  校验输出 → 保存车端 CSV/XML
           <──8 字节最终状态─────
```

**首次部署与地面平台接入请按 [TCP 对接与执行手册](tools/ground_vehicle_tcp/README.md) 的步骤执行**，其中包含配置、坐标换算、完整字段表、客户端代码、超时处理及排障。

## 服务流程

1. 客户端发送 32 字节规划请求。
2. 车端校验帧长度、字段和 CRC8。
3. 首次请求或地图指纹变化时，车端依次执行无 RViz 的 ROS 点云转换和 PGM 保存。
4. 车端将局部 UTM 坐标反投影为 WGS84，再由规划器转换到地图局部 ENU 坐标进行规划；请求中的起点和目标点 yaw 当前只接收、不参与规划。
5. 三组 CSV 和三个 `road_map*.xml` 都校验通过后，车端逐文件原子替换到 `output/`，再发送 8 字节成功响应。

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
bash scripts/run_vehicle.sh --config tools/ground_vehicle_tcp/config.yaml
```

systemd 部署前，先将服务文件中的 `WorkingDirectory` 和 `ExecStart` 中的 `/opt/muti_path_creater` 都改成车端实际项目路径，并配置为部署依赖的普通用户 `User=你的账号`，并确认该路径下已编译生成 `devel/setup.bash`；然后执行：

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
