# muti_path_creater

本工程运行在车辆工控机上，使用点云地图生成二维 PGM 地图，并根据地面车辆客户端发送的局部 UTM 起点、目标点生成三组轨迹。

当前唯一的 TCP 服务是 `tools/ground_vehicle_tcp`，协议采用 `docs/TCP通信协议/通信协议.xls` 定义的定长二进制协议。旧的 JSON `vehicle_gateway` 和 PySide6 `ground_station` 已移除，不再使用。

## 新工控机部署（先安装依赖，再编译）

原生部署脚本面向 **Ubuntu 20.04 + ROS1 Noetic**。在无人车上以普通账号执行，不要对整个脚本使用 `sudo`；安装系统包时脚本会调用 sudo。需要联网下载 apt、rosdep 和 Python 依赖。

```bash
cd /你的路径/2.5d-path-creater
bash scripts/deploy_vehicle.sh
```

如果新机器尚未配置 ROS 软件源，使用以下命令，允许脚本添加官方签名的 ROS1 apt 源并安装 Noetic：

```bash
bash scripts/deploy_vehicle.sh --setup-ros-repo
```

脚本会检查系统/ROS 环境，安装编译工具与系统库，使用 rosdep 根据工作区所有 `package.xml` 安装依赖，补齐 Python 依赖，再执行 `catkin_make` 并检查关键节点。默认只用 2 个编译任务，避免 PCL 编译占满内存。NumPy 限制为 1.x，以兼容 ROS/OpenCV；Python 依赖装到当前用户，部署和运行应使用同一个账号及 `/usr/bin/python3`，先退出 conda/virtualenv。

```bash
# 内存较小的工控机
bash scripts/deploy_vehicle.sh --jobs 1
# 分开安装与编译
bash scripts/deploy_vehicle.sh --deps-only
bash scripts/deploy_vehicle.sh --build-only --jobs 2
```

部署日志保存在 `log/deploy/`。脚本遇到 apt、网络、rosdep 或编译失败会停止，不会忽略错误继续运行；修复原因后可重复执行。`--build-only` 不安装依赖、不访问网络。脚本不删除旧编译目录、不改地图和原点配置、不启动 TCP 服务，也不安装 systemd 服务。

若项目从别的机器复制过来且携带 `build/`、`devel/`，出现 CMake 路径不一致时，先将这两个目录移走备份，再重新执行脚本。下载源码本身通常不含这些目录。

**Ubuntu 22.04/24.04 不能直接套用这份原生部署流程**：工程使用 ROS1/catkin 和旧版 TBB API，安装 ROS2 不能补齐这些依赖。脚本会提前退出，避免混装 focal 软件包；应使用 Ubuntu 20.04 环境/容器，或另行完成版本适配。Noetic 已结束常规支持，rosdep 更新显式包含 EOL 发行版。若 apt 报找不到包，检查 ROS/Ubuntu 软件源及网络；若 rosdep 更新失败，检查对 GitHub raw 和 ROS 索引的访问，不要用跳过依赖的方式继续编译。[rosdep 官方说明](https://github.com/ros-infrastructure/rosdep/blob/master/doc/overview.rst)。

编译完成后，继续按 [TCP 部署与对接手册](tools/ground_vehicle_tcp/README.md) 准备点云地图、填写原点并启动服务：

```bash
source devel/setup.bash
/usr/bin/python3 -m tools.ground_vehicle_tcp.main --config tools/ground_vehicle_tcp/config.yaml
```

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
source devel/setup.bash
python3 -m tools.ground_vehicle_tcp.main \
  --config tools/ground_vehicle_tcp/config.yaml
```

systemd 部署前，先将服务文件中的 `WorkingDirectory=/opt/muti_path_creater` 改成车端实际项目路径，并确认该路径下已编译生成 `devel/setup.bash`；然后执行：

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
