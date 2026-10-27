# 无人车 TCP 规划服务：部署与地面端对接手册

## 1. 两端分别做什么

本程序部署在**无人车工控机**上，是 TCP 服务端，默认监听 `0.0.0.0:8079`。地面控制平台是客户端，主动连接**无人车实际 IP**，发送规划请求。

流程如下：

```text
部署准备：将 ENU 点云 .pcd 和对应原点经纬高提前放到车端
地面端：填写起点、终点、yaw、规划超时 → 编码为 32 字节 → TCP 发送
车端：校验帧 → 读取本地 PCD → grid_map 高程/可通行栅格 → PGM/YAML
      → 起终点坐标转换 → 生成三条全局轨迹 → 校验并保存 CSV/XML
地面端：接收 8 字节最终响应 → 校验 → 显示成功或失败及错误原因
```

功能边界：

- **没有 PCD 上传命令**：地图通过 SCP、SFTP、U 盘等单独部署，不能把 PCD 内容直接发送到本 TCP 端口。
- **没有轨迹下载命令**：TCP 仅返回状态，轨迹文件保存在车端；需要地面端取文件时另接文件传输渠道。
- **没有进度、任务 ID、取消任务、地图选择或车辆启动命令**，目前仅支持 `0x0001` 规划请求。
- 规划成功表示输出已生成，不表示车辆已开始行驶。循迹与底盘控制需要下游模块接入。
- 起终点 yaw 会解码，但目前不参与规划约束。

## 2. 第一步：准备车端环境和地图

以下车端示例使用项目目录 `/home/gzq/2.5d-path-creater`；部署到其他目录时替换为实际路径。

首次部署推荐先在项目根目录以普通用户运行自动部署脚本，具体选项和系统要求见 [根目录部署说明](../../README.md)：

```bash
bash scripts/deploy_vehicle.sh
# 尚未配置 ROS1 软件源时：
# bash scripts/deploy_vehicle.sh --setup-ros-repo
```

脚本成功后可跳过下面的手动安装、编译步骤，直接准备地图；脚本自动支持 Ubuntu 18.04 + Melodic 和 Ubuntu 20.04 + Noetic；18.04 的环境隔离、启动及排障详见 [18.04 部署手册](../../docs/deployment-ubuntu18.04.md)。

下面是 20.04/Noetic 的手动安装方式；18.04 请使用上面的部署脚本，不要使用系统 Python 3.6 安装规划器。

需要 ROS1/catkin 环境、可运行的 `grid_map_pcl` 和 `grid_map_demos` 包，以及 Python 3 依赖。项目 Python 依赖见根目录 `requirements.txt`。

```bash
# 在无人车执行，先加载实际安装的 ROS1 环境
# 例如安装 ROS Noetic 时：source /opt/ros/noetic/setup.bash
cd /home/gzq/2.5d-path-creater
python3 -m pip install -r requirements.txt
# 首次部署或修改 ROS 源码后编译；已有有效编译产物可跳过
catkin_make
source devel/setup.bash
rospack find grid_map_pcl
rospack find grid_map_demos
```

将地图放到车端项目的 `map/pcd/`。例如从地面端执行以下 SCP 命令（需要自行提供车端 SSH 账号和地址）：

```bash
scp /path/to/scans_Global_ENU.pcd vehicle_user@192.168.1.100:/home/gzq/2.5d-path-creater/map/pcd/scans_Global_ENU.pcd
```

这里的 IP、账号、源文件路径均为示例。目标目录须事先存在，文件须完成复制后再发规划请求。

编辑车端 `tools/path_creater/config/map_enu.meta.yaml`，填写**生成该点云时实际采用的 ENU 原点**，不能直接沿用其他地图的原点：

```yaml
enu_origin:
  latitude: 22.39758935
  longitude: 113.30942461
  altitude: 25.8618
```

以上数值是仓库当前配置示例。PCD 内部应为以此原点建立的局部 ENU 米制坐标，而不是经纬度或绝对 UTM 坐标。

## 3. 第二步：检查车端配置

编辑 `tools/ground_vehicle_tcp/config.yaml`：

```yaml
host: 0.0.0.0
port: 8079
project_root: ../..
data_root: var/ground_vehicle_tcp
pcd_path: map/pcd/scans_Global_ENU.pcd
pcd_name: scans_Global_ENU.pcd
metadata_path: tools/path_creater/config/map_enu.meta.yaml
planner_config_path: tools/path_creater/config/config.yaml
conversion_timeout_s: 1800
```

- `host: 0.0.0.0` 表示监听所有本地网卡；地面客户端应填写无人车的实际 IP，不能填写 `0.0.0.0`。地面端填写 `127.0.0.1` 会连接地面端自己。
- `project_root` 相对于服务配置文件所在目录解析；其余这些文件/数据路径相对于项目根目录解析。
- `pcd_path` 用于源文件校验和缓存指纹；`pcd_name` 必须是与其文件名一致的 `.pcd` 文件名，不能包含目录。
- **当前转换器只向 ROS launch 传递 `pcd_name`，未传递 `pcd_path` 的目录**。ROS 默认从 `src/grid_map/grid_map_pcl/launch/grid_map_pcl_loader_node.launch` 中的 `folder_path` 读取，即项目 `map/pcd/`。建议保留此目录，仅同时修改 `pcd_path` 和 `pcd_name` 的文件名。迁移到其他地图目录时还必须同步修改 launch 的 `folder_path`，确保实际转换的文件与指纹校验的文件相同。
- `conversion_timeout_s` 为地图转换等待限时；请求中的 `timeout_s` 只限制规划子进程。自动转换会依次快速启动高程图、滤波和保存器，节点自行等待上游话题；服务端只等待保存器生成有效的 PGM/YAML，不再逐个用 `rostopic echo` 阻塞中间阶段。
- 规划器其他算法参数在 `tools/path_creater/config/config.yaml` 中配置；TCP 请求会覆盖本次任务的起终点和地图输入。

地图缓存位于 `var/ground_vehicle_tcp/maps/`。首次请求才转换地图；后续请求校验缓存后可复用，服务重启也可复用。PCD 内容、文件名、原点、代码中记录的转换版本/配置或缓存文件变化会触发重建。指纹不会自动散列所有 ROS 参数文件；手工修改滤波/转换参数后应在服务停止且无任务时清理对应地图缓存，或维护转换版本使其失效。

## 4. 第三步：启动车端服务

在无人车终端执行：

```bash
cd /home/gzq/2.5d-path-creater
bash scripts/run_vehicle.sh --config tools/ground_vehicle_tcp/config.yaml
```

保持终端运行。地图转换由服务收到请求后自动启动，无需再手动启动一套转换节点；避免其他同名 ROS 节点和同名输出话题干扰。ROS master 可预先启动，也可由本机 roslaunch 按 ROS 环境配置启动。

在车端另开终端检查监听：

```bash
ss -ltnp 'sport = :8079'
```

确保地面端可访问车端 IP 的 TCP 8079。协议不带认证，应仅向可信车地网络开放。

### 可选：部署为 systemd 服务

先手动联调成功，再部署开机启动。仓库服务模板已按本部署配置为账号 `union`、项目路径 `/home/union/2.5d-path-creater`；如果实际路径不同，安装前仍需同步修改 `User`、`WorkingDirectory` 和 `ExecStart`：

```bash
sudo install -m 0644 tools/ground_vehicle_tcp/ground-vehicle-tcp.service \
  /etc/systemd/system/ground-vehicle-tcp.service
```

`ExecStart` 以普通账号运行 `scripts/run_vehicle.sh`，自动选择 ROS 和规划解释器并加载工作空间。ROS master 尚未运行时，由自动转换的首次 `roslaunch` 创建；如果使用远端 ROS master，请在该服务的未提交本地配置或 systemd drop-in 中设置与手动环境一致的 `ROS_MASTER_URI`、`ROS_IP` 等变量。

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now ground-vehicle-tcp.service
sudo systemctl status ground-vehicle-tcp.service
sudo journalctl -u ground-vehicle-tcp.service -f
```

手动服务和 systemd 服务不能同时占用同一个端口。

## 5. 第四步：统一地面端和车端坐标定义

请求的 `x/y` 是**相对地图原点 UTM 投影位置的局部 UTM 偏移**，不是绝对 UTM，也不是 PCD 中的 ENU，更不是经纬度。

地面端从 WGS84 经纬度得到请求坐标的规则：

1. 读取与车端一致的 `enu_origin`。
2. 使用原点经度确定 UTM 分区：`zone = min(60, floor((longitude + 180) / 6) + 1)`。
3. 由原点纬度确定半球；北半球 EPSG 为 `32600 + zone`，南半球为 `32700 + zone`。
4. 在**同一分区、同一半球**中投影原点和起终点，得到东向 `E`、北向 `N`。
5. `x_m = E_point - E_origin`，`y_m = N_point - N_origin`。
6. 编码 `x_mm = round(x_m * 1000)`，`y_mm = round(y_m * 1000)`。

可复用 `tools/path_creater/geo.py` 的 `geodetic_to_local_utm(point, origin)`。例如在已安装依赖的项目根目录执行：

```python
from tools.path_creater.geo import GeoPoint, geodetic_to_local_utm
from tools.path_creater.map_metadata import load_map_metadata

origin = load_map_metadata("tools/path_creater/config/map_enu.meta.yaml").enu_origin
# 替换成实际起点的 WGS84 纬度、经度和高度
point = GeoPoint(latitude=22.39760, longitude=113.30945, altitude=origin.altitude)
x_m, y_m, _ = geodetic_to_local_utm(point, origin)
print(x_m, y_m)  # 作为客户端 --start-x / --start-y 的米制输入
```

如果地面平台持有 PCD 中的 ENU 点，应先按地图原点将 ENU 转为 WGS84，再转为上述局部 UTM。不要直接将 ENU 的 x/y 当作协议 x/y。

车端按“局部 UTM → WGS84 → 地图 ENU → 栅格”规划。请求没有高度字段，转换时采用地图原点高度。起终点必须处于地图可通行区域，未知区域默认不可通行。

## 6. 第五步：先用仓库客户端联调

**18.04 说明**：本节和下文 Python 示例应使用 `env -u PYTHONPATH -u PYTHONHOME .runtime/planner/bin/python` 替换 `python3`；车端服务使用 `bash scripts/run_vehicle.sh`。地面客户端与 ROS 无关，只需支持当前代码的 Python 环境（推荐 Python 3.8 或更新版本）。

### 6.1 车端本机验证

在车端另一终端、项目根目录执行：

```bash
python3 -m tools.ground_vehicle_tcp.client \
  --host 127.0.0.1 --port 8079 \
  --start-x 0 --start-y 0 --start-yaw 0 \
  --goal-x 10 --goal-y 5 --goal-yaw 0 \
  --timeout 120 --response-timeout 2400
```

### 6.2 地面端跨机器验证

地面电脑需要 Python 3 和仓库中的 `tools/ground_vehicle_tcp` 包；上述客户端仅使用 Python 标准库及同包协议模块，无需地面电脑运行 ROS。将代码复制到地面端后，在对应项目根目录执行：

```bash
python3 -m tools.ground_vehicle_tcp.client \
  --host 192.168.1.100 --port 8079 \
  --start-x 0 --start-y 0 --start-yaw 0 \
  --goal-x 10 --goal-y 5 --goal-yaw 0 \
  --timeout 120 --response-timeout 2400
```

把 `192.168.1.100` 换成无人车 IP。两条命令的坐标都是示例，必须换成地图内已确认可通行的实际起终点；示例点失败不代表 TCP 通信失败。

客户端参数 `x/y` 单位是米，yaw 单位是弧度；客户端会乘 1000 编码。yaw 目前可统一填写 0。客户端会在标准错误实时输出连接、发送和等待状态，默认每 10 秒输出一次等待心跳；可用 `--progress-interval` 调整间隔。协议没有处理中间态响应，因此这些输出只表示本地 TCP 请求仍在等待，不能用于判断服务端具体转换阶段。最终结果仍输出到标准输出：成功打印 `success=True error_code=0x00`，退出码 0；服务端报告失败时退出码 1；连接、等待或协议异常时退出码 2。

首次请求可能包含较长地图转换。`--timeout 120` 是规划限时；`--response-timeout 2400` 是客户端等待响应时间，应覆盖转换限时 1800 秒、规划 120 秒以及文件哈希、启动和输出校验等开销。

## 7. 地面控制软件应如何实现 TCP 对接

### 7.1 请求字段（总长 32 字节）

协议依据 `docs/TCP通信协议/通信协议.xls`，编解码实现以 `protocol.py` 为准。所有多字节字段均为**大端序**，有符号整数使用 int32 二进制补码。

| 字节偏移（从 0 起） | 长度 | 字段 | 类型、单位、取值 |
| --- | --- | --- | --- |
| 0 | 2 | 请求帧头 | `0xFFDA`，实际字节 `FF DA` |
| 2 | 2 | 消息类型 | uint16，`0x0001`：规划 |
| 4 | 1 | 总帧长 | uint8，`0x20`，即 32 |
| 5 | 4 | 起点 x | int32，局部 UTM，毫米 |
| 9 | 4 | 起点 y | int32，局部 UTM，毫米 |
| 13 | 4 | 起点 yaw | int32，毫弧度，当前忽略 |
| 17 | 4 | 终点 x | int32，局部 UTM，毫米 |
| 21 | 4 | 终点 y | int32，局部 UTM，毫米 |
| 25 | 4 | 终点 yaw | int32，毫弧度，当前忽略 |
| 29 | 2 | 规划超时 | uint16，秒，1～65535 |
| 31 | 1 | CRC8 | 对偏移 0～30 的 31 字节计算 |

int32 范围是 `-2147483648～2147483647`；地面端换算后须检查溢出。发送原始二进制字节，不是 JSON、文本数字、十六进制字符串，也不要追加换行符。

### 7.2 响应字段（总长 8 字节）

| 字节偏移 | 长度 | 字段 | 类型、取值 |
| --- | --- | --- | --- |
| 0 | 2 | 响应帧头 | `0xFFDB`，实际字节 `FF DB` |
| 2 | 2 | 消息类型 | uint16，`0x0001` |
| 4 | 1 | 总帧长 | uint8，`0x08` |
| 5 | 1 | 结果 | 0：失败；1：成功 |
| 6 | 1 | 错误码 | 成功时必须为 `0x00` |
| 7 | 1 | CRC8 | 对偏移 0～6 的 7 字节计算 |

CRC 参数：初值 `0x00`，多项式 `0x07`，输入/输出不反射，最终不异或。先异或每个输入字节，再按最高位判断、左移和多项式异或，始终保留低 8 位。CRC 不包含 CRC 字节本身。可复用 `protocol.py` 中的 `crc8`。

| 错误码 | 含义 | 地面端处理建议 |
| --- | --- | --- |
| `0x00` | 成功 | 提示轨迹已生成，允许后续模块读取车端输出 |
| `0x01` | 目标点超出地图 | 修正目标点或核对坐标系 |
| `0x02` | 目标不可通行或无可用路径 | 更换目标或检查障碍、未知区域 |
| `0x03` | 起点越界或不可通行 | 修正起点或核对坐标系 |
| `0x04` | 地图准备/转换失败 | 检查 PCD、原点、ROS 环境及转换日志 |
| `0x05` | 规划子进程超时 | 检查地图和参数，必要时增大规划限时 |
| `0x06` | 帧、消息类型、字段或 CRC 错误 | 检查编码、大小端和 CRC |
| `0x07` | 内部/输出错误或服务忙 | 检查车端日志，避免并发请求 |

### 7.3 连接与请求状态机

1. 客户端连接车端实际 IP 和端口，设置连接超时与独立的响应等待超时。
2. 完成坐标换算和字段范围检查，组装 32 字节帧，计算 CRC，循环发送直至全部字节发出。
3. 将界面置为“规划中”，异步等待响应，避免阻塞地面界面线程。
4. TCP 是字节流：先累计读取 5 字节头，再按长度读取剩余 3 字节；不能假设一次 `recv()` 就是完整响应。断开但未收到完整帧应视为通信失败。
5. 校验帧头、类型、长度、CRC，以及 result 与错误码的一致性。只有 `result=1 && error_code=0` 才是规划成功。
6. 收到最终响应后才允许发下一条请求。可复用同一连接，也可每次新建连接；仓库客户端每次请求新建连接。
7. 一次仅执行一个任务，其他连接在服务忙时会收到 `0x07`；同一连接上的请求会顺序处理。没有任务 ID，禁止流水线连续提交请求。
8. 本地等待超时/断网只能说明没有确认结果，**不会取消车端正在执行的任务**。不要自动高频重试；应先检查车端任务与日志，再决定重发。连接成功也不等于规划成功。

### 7.4 可直接复用的 Python 调用

以下代码在项目根目录运行。地面控制平台使用 Python 时可直接集成；C++、C# 等按上面的字段表实现同等编码与累计接收逻辑。

```python
from tools.ground_vehicle_tcp.protocol import NavigationRequest, encode_request
from tools.ground_vehicle_tcp.client import request

# 示例：局部 UTM 起点 (0, 0) 米、终点 (10, 5) 米
navigation = NavigationRequest(
    start_x_mm=0,
    start_y_mm=0,
    start_yaw_millirad=0,
    goal_x_mm=10000,
    goal_y_mm=5000,
    goal_yaw_millirad=0,
    timeout_s=120,
)
try:
    success, error_code = request(
        host="192.168.1.100",  # 替换为无人车 IP
        port=8079,
        frame=encode_request(navigation),
        connect_timeout_s=5,
        response_timeout_s=2400,
    )
    print("规划成功" if success else f"规划失败：0x{error_code:02X}")
except (OSError, ValueError) as exc:
    print(f"通信或协议异常，任务结果未确认：{exc}")
```

## 8. 第六步：确认和使用车端输出

成功响应发出前，车端会检查九个文件非空、CSV 表头和数值有效、XML 可解析且顶点坐标有效，再逐文件原子替换到规划配置中的 `output_dir`（默认项目根目录 `output/`）。

| 策略 | 经纬度 CSV（表头 latitude,longitude） | ENU CSV（表头 x,y） | XML |
| --- | --- | --- | --- |
| 距离优先 | `traj_distance_geo.csv` | `traj_distance_enu.csv` | `road_map2.xml` |
| 平滑优先 | `traj_smooth_geo.csv` | `traj_smooth_enu.csv` | `road_map3.xml` |
| 综合代价/安全距离优先 | `traj_cost_aware_geo.csv` | `traj_cost_aware_enu.csv` | `road_map1.xml` |

CSV 经纬度使用角度，ENU 使用米。XML 的 `x/y` 为局部 UTM 偏移、单位米，与请求的坐标定义一致；XML yaw 由轨迹方向生成。

在车端检查：

```bash
cd /home/gzq/2.5d-path-creater
ls -lh output/traj_*.csv output/road_map*.xml
ls -lt var/ground_vehicle_tcp/jobs/
```

每个任务保存在 `var/ground_vehicle_tcp/jobs/<内部任务目录>/`，含 `job.log`、本次规划配置和输出；该内部目录名不会通过 TCP 回传。地图准备早期失败时日志可能不完整，需结合服务终端或 systemd 日志排查。

**必须等到本次成功响应后才读取标准输出文件**。原子替换只保证单个文件完整，九个文件不是一次事务；失败时标准目录可能仍是旧结果，或包含部分已替换文件，不能仅凭文件存在判断本次成功。后续请求还会覆盖标准文件，需要保留结果时应在下一次请求前归档，并协调下游读取。

若地面端需要查看这些文件，可在确认成功、且没有后续任务覆盖时通过 SFTP/SCP 获取；这属于额外文件通道，不属于本 TCP 协议。

## 9. 常见问题与联调验收

| 现象 | 检查方向 |
| --- | --- |
| 连接被拒绝 | 服务是否启动、IP/端口是否正确、是否绑定到可达网卡 |
| 连接超时 | 车地网络、路由、防火墙是否允许 TCP 8079 |
| 首次响应慢 | 正在进行地图转换；检查任务日志并给足响应等待时间 |
| `0x04` | PCD 是否已部署、文件名/ROS 目录是否一致、原点有效、ROS 包是否可用 |
| `0x01/0x03` | 是否误传绝对 UTM、ENU 或经纬度；起终点是否越界 |
| `0x02` | 目标是否落在障碍/未知区域、地图是否连通 |
| `0x06` | 是否发送了文本、长度或大小端错误、CRC 是否包括了自身 |
| `0x07` | 是否多端同时规划、输出目录权限/磁盘空间是否正常 |
| 客户端超时但车端仍在运行 | 本地超时不会取消任务，先查日志再重试 |
| 收到失败但 output 仍有文件 | 可能是旧结果，不能作为本次成功依据 |

建议按顺序验收：车端环境和地图就绪 → 端口监听正常 → 使用有效地图点本机规划成功 → 地面端用同一组坐标远程规划成功 → 自研地面客户端发出等价帧并正确处理成功/错误响应 → 对接车端轨迹消费模块。

仅修改协议或客户端代码时，可运行现有测试：

```bash
python3 -m unittest tests.test_ground_vehicle_tcp -v
python3 -m compileall -q tools/ground_vehicle_tcp
```

这些测试不代替真实 ROS、地图和跨机器网络验收。
