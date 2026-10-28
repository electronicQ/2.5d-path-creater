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

启动前，先按下一节将 PCD 地图和原点元数据放到工控机项目目录中，再使用统一入口启动（18.04 不要直接运行系统 python3）：

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

## 启动前必须准备的两个地图文件

**首次启动规划服务前，必须提前将对应的 PCD 点云地图和 ENU 原点经纬高文件放到工控机上。** 已创建的 `map/pcd/` 目录用于存放点云，原点文件放在 `tools/path_creater/config/` 中。

`tools/path_creater/config/config.yaml` 由 Git 跟踪，规划参数修改需要通过 Git 提交管理；`map_enu.meta.yaml` 是工控机本地地图原点，已被 Git 忽略，不会被 `git pull` 覆盖。新检出的仓库可复制地图原点模板：

```bash
cp tools/path_creater/config/map_enu.meta.example.yaml \
  tools/path_creater/config/map_enu.meta.yaml
```

`map/pcd/` 和 `map/pgm/` 同样是工控机本地数据目录，已被 Git 忽略。`map/pcd/` 保存实际 PCD，`map/pgm/` 保存手动转换出的 PGM/YAML；`git pull` 不会覆盖这两个目录中的内容。新环境需要先创建目录并放入实际地图：

```bash
mkdir -p map/pcd map/pgm
```

以下路径均相对于项目根目录：

```text
2.5d-path-creater/
├── map/
│   └── pcd/
│       └── scans_Global_ENU.pcd       # 放入实际使用的点云地图
└── tools/
    └── path_creater/
        └── config/
            └── map_enu.meta.yaml     # 填写该点云对应的 ENU 原点经纬高
```

例如，工控机上的项目目录为 `/home/union/2.5d-path-creater`，则两个文件分别位于：

| 文件 | 工控机上的完整路径 |
| --- | --- |
| PCD 点云 | `/home/union/2.5d-path-creater/map/pcd/scans_Global_ENU.pcd` |
| 原点经纬高 | `/home/union/2.5d-path-creater/tools/path_creater/config/map_enu.meta.yaml` |

`map_enu.meta.yaml` 使用以下格式；数值仅示意，必须替换为**生成这份点云时实际采用的原点**，不能直接沿用仓库示例：

```yaml
enu_origin:
  latitude: 22.39758935
  longitude: 113.30942461
  altitude: 25.8618
```

`latitude` 为纬度、`longitude` 为经度，单位为度；`altitude` 为高度，单位为米。PCD 内部坐标应为相对于该原点的局部 ENU 米制坐标。原点文件与点云必须配套。

检查 `tools/ground_vehicle_tcp/config.yaml` 中的配置与实际文件一致：

```yaml
pcd_path: map/pcd/scans_Global_ENU.pcd
pcd_name: scans_Global_ENU.pcd
metadata_path: tools/path_creater/config/map_enu.meta.yaml
```

如果点云叫 `campus.pcd`，仍放入 `map/pcd/`，并同时把 `pcd_path` 改为 `map/pcd/campus.pcd`、`pcd_name` 改为 `campus.pcd`。两个配置中的文件名必须一致。建议保留 `map/pcd/` 目录，因为当前 ROS 转换器默认从这里读取点云；仅修改 `pcd_path` 的目录并不能改变 ROS 的读取目录。

在项目根目录执行以下命令可检查默认文件是否存在；使用其他文件名时同步修改命令中的路径：

```bash
ls -lh map/pcd/scans_Global_ENU.pcd
cat tools/path_creater/config/map_enu.meta.yaml
```

点云需要通过 U 盘、SCP 或 SFTP 等方式提前复制到工控机。当前 TCP 接口只接收规划请求，不负责上传这两个文件；部署脚本也不会生成真实点云或自动确定地图原点。

## 手动调试

以下以 **Ubuntu 18.04 + ROS Melodic** 为例，按“PCD → 高程栅格 → 可通行栅格 → PGM/YAML → 轨迹”的顺序逐步验证。先完成上述自动部署，并准备好点云和原点文件。示例工程目录为 `/home/union/2.5d-path-creater`，放在其他位置时修改各命令中的 `cd` 路径即可，无需修改保存脚本。

手动调试期间先停止 TCP 服务，避免它启动同名的地图转换节点。如果通过 systemd 启动了服务，执行 `sudo systemctl stop ground-vehicle-tcp.service`；如果在终端运行了 `scripts/run_vehicle.sh`，在对应终端按 Ctrl+C。

### 1. 各 ROS 终端加载环境

下面的终端 A、B、C **都先执行一次**：

```bash
# 如果终端显示 (base)，先执行 conda deactivate；有多层环境时重复退出
cd /home/union/2.5d-path-creater
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```

若刚更新了滤波代码或尚未以 Release 编译，先在项目根目录执行一次，完成后重新加载 `devel/setup.bash`：

```bash
bash scripts/deploy_vehicle.sh --build-only --jobs 2
source devel/setup.bash
```

### 2. 终端 A：将 PCD 转为高程栅格

手动转换默认使用 `scans_Global_ENU.pcd`，文件应位于项目的 `map/pcd/` 中；如需转换其他文件，显式传入 `pcd_filename`。

```bash
ls -lh map/pcd/scans_Global_ENU.pcd
roslaunch grid_map_pcl grid_map_pcl_loader_node.launch \
  pcd_filename:=scans_Global_ENU.pcd \
  enable_rviz:=false publish_point_cloud:=false
```

该命令会自动启动本地 ROS master（尚未运行时）、读取点云并生成高程栅格。看到 `Saving grid map successful: true` 表示这一阶段完成。默认还会保存 `map/pcd/output_grid_map.bag`，并通过 `/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud` 发布高程地图。

**保持终端 A 运行**，后续滤波节点需要接收它发布的地图。这里尚未生成规划需要的 PGM。

### 3. 终端 B：先启动地图保存器，等待数据

在终端 B 加载第 1 步的 ROS 环境后执行：

```bash
/usr/bin/python2.7 tools/pgm_saver/save_continuous_map.py -n map_enu
```

出现“等待地图数据”是正常状态。保存器订阅 `/grid_map_visualization/traversability_grid`，此时先等待下一步输出。不要使用 Conda 的 `python` 运行它。

必须先启动保存器再启动无 RViz 的滤波流程：可视化节点在有输出订阅者时才会订阅并转换地图，保存器提供所需的占据栅格订阅。

### 4. 终端 C：滤波并发布可通行栅格

在终端 C 加载第 1 步的 ROS 环境后执行：

```bash
roslaunch grid_map_demos filters_demo_gai.launch enable_rviz:=false
```

该步骤执行平滑、法向量和可通行性等滤波，将结果交给可视化节点转换为占据栅格。滤波耗时日志可用于判断具体停在哪个阶段；大地图仍可能需要数分钟，详见 [18.04 大地图排障](docs/deployment-ubuntu18.04.md#8-大地图滤波长时间运行先检查编译优化)。如需 RViz 查看，可将 `enable_rviz` 改为 `true`，但大地图显示会增加开销。

终端 B 收到数据后会保存文件并自动退出：

```text
/home/union/2.5d-path-creater/map/pgm/map_enu.pgm
/home/union/2.5d-path-creater/map/pgm/map_enu.yaml
```

在终端 B 检查：

```bash
ls -lh map/pgm/map_enu.pgm map/pgm/map_enu.yaml
cat map/pgm/map_enu.yaml
```

`-n map_enu` 与下一步默认规划配置中的文件名对应。保存器自动依据工程位置确定目录；不要写死 `/home/union/...`。生成的 `map_enu.yaml` 保存栅格分辨率和局部地图原点，**原点经纬高另由 `map_enu.meta.yaml` 提供**，两个 YAML 用途不同。

确认保存成功后，可以在终端 C、A 分别按 Ctrl+C 释放资源；后续离线规划不依赖 ROS 节点继续运行。

### 5. 配置规划地图与实际起终点

编辑 `tools/path_creater/config/config.yaml`，保留已有规划参数，核对以下字段：

```yaml
map_yaml: ../../../map/pgm/map_enu.yaml
map_meta: map_enu.meta.yaml
output_dir: ../../../output
```

这里的相对路径以配置文件所在目录 `tools/path_creater/config/` 为基准。`map_meta` 对应前面准备的 `tools/path_creater/config/map_enu.meta.yaml`，必须与点云的 ENU 原点一致。

再将配置中的 `start` 和 `goal` 改为实际起终点的 `latitude`、`longitude`、`altitude`：经纬度单位为度，高度单位为米，不能直接沿用仓库示例。**离线规划配置接收经纬高；TCP 客户端接收局部 UTM 米制偏移，两种输入不能直接混用。** 起终点应在地图内，并留出配置中障碍膨胀所需的通行空间。

`tracking_sampling` 控制最终 XML 的轨迹点密度。默认直线最大约 `10 m` 一点、缓弯约 `1～2 m`、急弯约 `0.3～0.8 m`；可通过 `max_spacing_m`、`min_spacing_m`、`chord_tolerance_m` 和 `max_heading_change_deg` 联合调整。

`corridor_centering` 只修改 `cost_aware` 对应的 `road_map1.xml`。默认 `corner_only: true`，只在检测到的转弯附近启用道路中心代价，直线段保持原规划；shortcut 和曲线圆角也只在这些转弯区域继续限制与高代价边界的距离。`road_map2.xml` 和 `road_map3.xml` 保持原逻辑。

### 6. 运行离线规划并查看输出

在项目根目录执行（可复用终端 B）：

```bash
cd /home/union/2.5d-path-creater
env -u PYTHONPATH -u PYTHONHOME .runtime/planner/bin/python \
  -m tools.path_creater.main --config tools/path_creater/config/config.yaml
```

该命令只读取已保存的地图并规划，不会重新转换 PCD，也不需要先启动 TCP 服务。成功后检查：

```bash
ls -lh output/road_map1.xml output/road_map2.xml output/road_map3.xml
ls -lh output/overlay.png
```

默认只输出三个 XML 和一张路径叠加图。`road_map1.xml`、`road_map2.xml`、`road_map3.xml` 分别是代价感知、距离优先、平滑优先轨迹，XML 使用局部 UTM 坐标和形状自适应轨迹点。`overlay.png` 使用同色细线连接同一条轨迹的相邻点；距离优先用红色圆点、平滑优先用绿色三角形、综合代价优先用蓝色正方形。不同尺寸的分层标记保证完全重叠时三种颜色仍可见。

只调整起终点或规划权重时，修改配置并重复第 6 步即可；更换点云或滤波参数时，需要重新执行第 2～4 步保存地图。手动生成的地图不会自动登记进 TCP 服务的地图缓存；恢复正常服务时先停止手动 ROS launch，再运行 `bash scripts/run_vehicle.sh`，服务按自身缓存流程处理请求。

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
5. 三个 `road_map*.xml` 都校验通过后，车端将三份 XML 和 `overlay.png` 逐文件原子替换到 `output/`，再发送 8 字节成功响应。

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
output/road_map1.xml
output/road_map2.xml
output/road_map3.xml
output/overlay.png
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

配置开机自启前，先停止手动占用 `8079` 的服务，并确认实际路径下已生成 `devel/setup.bash` 和 `.runtime/planner/bin/python`。仓库的 systemd 单元已配置为账号 `union`、项目路径 `/home/union/2.5d-path-creater`；若部署位置或账号不同，先同步修改 `User`、`WorkingDirectory` 和 `ExecStart`：

```bash
sudo install -m 0644 \
  tools/ground_vehicle_tcp/ground-vehicle-tcp.service \
  /etc/systemd/system/ground-vehicle-tcp.service
sudo systemctl daemon-reload
sudo systemctl enable --now ground-vehicle-tcp.service
systemctl status ground-vehicle-tcp.service --no-pager
journalctl -u ground-vehicle-tcp.service -f -n 100
```

服务启动后可用 `ss -ltnp 'sport = :8079'` 确认监听。服务随系统开机启动，异常退出后会自动重启；修改代码或配置后执行 `sudo systemctl restart ground-vehicle-tcp.service`。

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
