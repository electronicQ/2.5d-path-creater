# muti_path_creater

`muti_path_creater` 是一个从无人机点云地图生成静态连续栅格地图，并基于该地图生成多条全局轨迹的工程。

主目标不是重新完成 SLAM 建图，而是把已有的局部 ENU `.pcd` 点云地图处理成可规划的静态地图，并从同一起点到同一终点生成三条不同策略的全局路径。

## 主流程

```text
无人机点云 .pcd
-> grid_map_pcl 生成 elevation 高程 GridMap
-> grid_map filter chain 生成 traversability 可通行层
-> grid_map_visualization 转为 nav_msgs/OccupancyGrid
-> tools/pgm_saver 保存连续灰度 .pgm + .yaml
-> tools/path_creater 读取静态地图并规划三条轨迹
-> 输出 ENU CSV 与经纬度 CSV
```

## 目录结构

| 路径 | 作用 |
| --- | --- |
| `src/grid_map/` | ROS/catkin 软件包，负责点云到高程地图、多层 GridMap、OccupancyGrid 的转换。 |
| `src/grid_map/grid_map_pcl/` | 读取 `.pcd` 点云，生成 `elevation` 高程层。 |
| `src/grid_map/grid_map_demos/` | 使用 filter chain 从高程层计算坡度、粗糙度、边缘和 `traversability`。 |
| `src/grid_map/grid_map_visualization/` | 将 GridMap 层发布为可视化消息或 `nav_msgs/OccupancyGrid`。 |
| `tools/pgm_saver/` | 订阅连续 OccupancyGrid 并保存为 `.pgm` 与 `.yaml`。 |
| `tools/path_creater/` | ROS-independent 离线全局规划器，读取静态地图并输出三条轨迹。 |
| `map/pcd/` | 建议存放原始或待处理点云地图。 |
| `map/pgm/` | 建议存放已生成的静态 `.pgm` + `.yaml` 地图。 |
| `output/` | 默认轨迹、叠加图和解析后配置输出目录。 |
| `config.yaml` | 离线规划器的默认配置文件。 |
| `docs/project-context.md` | 项目长期背景、坐标系和地图语义约定。 |

## 环境依赖

ROS 部分依赖 catkin 工作区和 `grid_map` 相关 ROS 包。离线规划部分依赖 Python：

```bash
python3 -m pip install -r requirements.txt
```

`requirements.txt` 当前包含：

- `numpy`
- `PyYAML`
- `Pillow`
- `scipy`

## 执行流程

以下命令默认从仓库根目录 `/home/cclg/muti_path_creater` 执行。

### 1. 准备 ROS 环境

```bash
source devel/setup.bash
```

如果重新编译过 `src/grid_map`，先完成 catkin 构建，再重新 source。

### 2. 点云转高程 GridMap

```bash
roslaunch grid_map_pcl grid_map_pcl_loader_node.launch
```

默认配置：

- launch 文件：`src/grid_map/grid_map_pcl/launch/grid_map_pcl_loader_node.launch`
- 点云参数：`src/grid_map/grid_map_pcl/config/parameters.yaml`
- 默认点云目录：`map/pcd`
- 默认点云文件：`auto`，自动选择 `map/pcd` 中最近修改的 `.pcd`
- 输出话题：`/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud`
- 默认层名：`elevation`

常用覆盖参数示例：

```bash
roslaunch grid_map_pcl grid_map_pcl_loader_node.launch pcd_filename:=changlilou_ENU.pcd
```

### 3. 高程 GridMap 转可通行 GridMap 与 OccupancyGrid

```bash
roslaunch grid_map_demos filters_demo_gai.launch
```

这一步会：

- 订阅 `/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud`
- 发布 `/grid_map_filter_demo/filtered_map`
- 使用 filter chain 计算 `slope`、`roughness`、`edges`、`traversability`
- 通过 `grid_map_visualization` 发布 `/grid_map_visualization/traversability_grid`

相关文件：

- `src/grid_map/grid_map_demos/launch/filters_demo_gai.launch`
- `src/grid_map/grid_map_demos/config/filters_demo_filter_chain_gai.yaml`
- `src/grid_map/grid_map_demos/config/filters_demo_gai.yaml`

### 4. 保存连续静态地图

在 `/grid_map_visualization/traversability_grid` 已经发布后运行：

```bash
python3 tools/pgm_saver/save_continuous_map.py -n example_map
```

输出：

```text
map/pgm/example_map.pgm
map/pgm/example_map.yaml
```

`-n` 只传裸文件名时默认保存到 `map/pgm`。如果传入 `custom/example_map` 这类显式相对路径，则按仓库根目录下的相对路径保存。

PGM 灰度语义：

- OccupancyGrid `-1` -> PGM `205`，未知区域
- OccupancyGrid `100` -> PGM `0`，不可通行或高占据
- OccupancyGrid `0` -> PGM `254`，最可通行
- `1..99` -> 连续灰度代价

### 5. 使用静态地图生成三条全局轨迹

先编辑 `config.yaml`，填入：

- `map_yaml`：静态地图 `.yaml`
- `map_meta`：该地图对应的局部 ENU 原点经纬高
- `start` / `goal`：起终点经纬度
- `unknown_gray`、`absolute_obstacle_gray` 和三组 planner 权重

然后运行：

```bash
python3 -m tools.path_creater.main --config config.yaml
```

更多规划器细节见 `tools/path_creater/README.md`。

## 坐标系约定

工程内同时存在三类坐标，不能混用：

- 经纬度：`latitude, longitude, altitude`
- 局部 ENU：`x, y, z`，单位为米
- 栅格索引：`row, col`

点云和静态地图使用局部 ENU 坐标。用户输入的起点和终点使用经纬度，规划器内部转换为 ENU 和栅格索引，最终同时输出经纬度轨迹和 ENU 轨迹。

## 未知区域约定

点云未观测区域必须保持未知语义：

```text
elevation NaN
-> traversability NaN
-> OccupancyGrid -1
-> PGM gray 205
```

后续如果需要补洞，只应作为显式、可配置的规划前处理步骤；默认不允许把未知区域当作自由空间。

## 轨迹输出

离线规划器默认生成三类轨迹：

- `distance`：距离优先
- `smooth`：转向更少、更平滑
- `cost_aware`：更重视高灰度可通行区域和障碍物安全距离

每类轨迹输出两份 CSV：

```text
traj_distance_geo.csv
traj_distance_enu.csv
traj_smooth_geo.csv
traj_smooth_enu.csv
traj_cost_aware_geo.csv
traj_cost_aware_enu.csv
```

`*_geo.csv` 用于后续循迹或跨 ENU 原点使用，`*_enu.csv` 用于地图对齐、调试和复现。

规划器会先去除栅格 A* 的冗余折点，再对路径做安全圆角曲线平滑，并按配置的间隔采样保存轨迹点。默认采样间隔为 `1.0 m`，曲线采样过程不会穿越未知区、障碍区或障碍膨胀区。

## 维护提示

- 根目录 README 只描述整个工程和端到端流程。
- `tools/path_creater/README.md` 只描述离线轨迹生成工具。
- `docs/project-context.md` 保存长期约定，涉及坐标系、未知区域和地图语义时优先参考它。
