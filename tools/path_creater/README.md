# path_creater

`tools/path_creater` 是一个不依赖 ROS 的离线全局规划器。它读取连续灰度 `.pgm` + `.yaml` 静态地图，根据经纬度起点和终点生成三条不同策略的全局轨迹。

## 输入与输出

输入：

- ROS map_server 风格的地图 `.yaml`
- 对应的灰度 `.pgm`
- 地图局部 ENU 原点元数据 `.meta.yaml`
- 经纬度起点 `start`
- 经纬度终点 `goal`

输出：

```text
output/traj_distance_geo.csv
output/traj_distance_enu.csv
output/traj_smooth_geo.csv
output/traj_smooth_enu.csv
output/traj_cost_aware_geo.csv
output/traj_cost_aware_enu.csv
output/overlay.png
output/config.resolved.yaml
```

CSV 格式：

- `*_geo.csv`：`latitude,longitude`
- `*_enu.csv`：`x,y`

默认会对 A* 输出先做直线可视化 shortcut，再做安全圆角曲线平滑，并按固定间隔采样；CSV 中保存的是最终采样点。

## 地图灰度语义

规划器把静态 `.pgm` 当作连续代价地图，不当作普通二值地图。

默认语义：

- `gray == 205`：未知区域，禁止通行
- `gray <= 90`：绝对障碍，禁止通行
- `gray > 90` 且 `gray != 205`：可参与规划
- 灰度越大，通行代价越低
- 灰度越小，通行代价越高

连续灰度代价使用单调映射：

```text
gray_cost = 1.0 - gray / 255.0
```

## 坐标转换

配置中的 `start` 和 `goal` 使用经纬度。规划内部流程是：

```text
latitude/longitude
-> WGS84/ECEF/ENU 转换
-> 局部 ENU x,y
-> 栅格 row,col
-> A* 搜索
-> ENU 轨迹
-> 经纬度轨迹
```

不要直接把经纬度当作栅格坐标，也不要把 `row,col` 当作 ENU 米制坐标。

## 配置文件

默认从仓库根目录读取 `config.yaml`：

```yaml
map_yaml: map/pgm/example_map.yaml
map_meta: map/pgm/example_map.meta.yaml

start:
  latitude: 30.000001
  longitude: 120.000001
  altitude: 0.0

goal:
  latitude: 30.000500
  longitude: 120.000600
  altitude: 0.0

unknown_gray: 205
absolute_obstacle_gray: 90
gray_cost:
  enabled: true
  weight: 1.0

inflation_radius_m: 0.6
d_safe_m: 1.0
smooth_paths: true
curve_smoothing:
  enabled: true
  sample_interval_m: 1.0
  corner_cut_fraction: 0.35
  max_corner_cut_m: 8.0
  min_corner_cut_m: 0.2
  safety_check_step_m: 0.25

planners:
  distance:
    w_turn: 0.0
    w_clear: 1.0
    w_gray: 0.5
  smooth:
    w_turn: 4.0
    w_clear: 2.0
    w_gray: 1.0
  cost_aware:
    w_turn: 0.5
    w_clear: 12.0
    w_gray: 35.0

output_dir: output
write_overlay: true
write_resolved_config: true
```

`map_meta` 文件只保存该静态地图对应的局部 ENU 原点：

```yaml
enu_origin:
  latitude: 30.0
  longitude: 120.0
  altitude: 0.0
```

## 运行

从仓库根目录执行：

```bash
python3 -m pip install -r requirements.txt
python3 -m tools.path_creater.main --config config.yaml
```

也可以使用独立配置文件：

```bash
python3 -m tools.path_creater.main --config path/to/your_config.yaml
```

相对路径会按配置文件所在目录解析。例如 `map_yaml: map/pgm/example_map.yaml` 是相对于当前 `config.yaml` 所在目录，而不是相对于 `tools/path_creater`。

## 三种规划策略

规划器使用 8 邻接 A*。每一步代价包含：

```text
step_length
+ w_turn * turn_penalty
+ w_clear * clearance_penalty
+ w_gray * gray_cost
```

三组默认策略：

- `distance`：更偏短路径
- `smooth`：更重视少转弯和平滑
- `cost_aware`：更重视高灰度区域和障碍物安全距离，即使绕远也更倾向于走通行程度高的区域

如果三条路径太像，可以拉大各策略权重差异，尤其是 `w_turn`、`w_clear` 和 `w_gray`。

`smooth_paths: true` 会在规划完成后做 line-of-sight shortcut smoothing：如果两个非相邻路径点之间的直线段全程处在膨胀后的自由栅格内，就移除中间冗余点。这个步骤不会穿越未知区、障碍区或障碍膨胀区，用来减少栅格搜索带来的锯齿折线。

`curve_smoothing.enabled: true` 会继续对 shortcut 后的路径做安全圆角化：每个折角会尝试用二次 Bezier 曲线替代，曲线采样点和相邻采样点连线都必须处在膨胀后的自由栅格内。最终 `*_enu.csv` 和 `*_geo.csv` 按 `sample_interval_m` 等弧长间隔保存点，默认间隔为 `1.0 m`。

## 常见失败原因

- `map_yaml` 指向的 `.yaml` 或 `.pgm` 不存在
- `map_meta` 不存在，或没有 `enu_origin`
- `start` / `goal` 仍是 `null`
- 起点或终点超出地图范围
- 起点或终点落在未知区、绝对障碍或膨胀后的障碍区
- `inflation_radius_m` 太大，导致自由空间被膨胀吃掉
- PGM 中未知区域没有保持为 `205`，导致地图语义不符合预期
- 地图没有保存到工程约定的 `map/pgm`，导致 `map_yaml` 路径写错

## 模块说明

| 文件 | 作用 |
| --- | --- |
| `main.py` | 命令行入口，串联配置、地图读取、坐标转换、规划和输出。 |
| `map_loader.py` | 读取 `.pgm` + `.yaml`，保留原始灰度和地图元数据。 |
| `map_semantics.py` | 把连续灰度图转换为 forbidden/free/gray_cost。 |
| `clearance.py` | 根据 forbidden/free 计算障碍膨胀和距离场。 |
| `planner.py` | 8 邻接 A* 和不同权重策略。 |
| `path_smoothing.py` | 对规划结果做可视线捷径平滑，减少不必要折点。 |
| `curve_smoothing.py` | 对 shortcut 路径做安全圆角曲线平滑，并按固定间隔采样。 |
| `geo.py` | WGS84、ECEF、ENU 坐标转换。 |
| `map_metadata.py` | 读取地图 ENU 原点元数据。 |
| `auto_endpoints.py` | 解析起终点并检查可通行性。 |
| `trajectory.py` | 写出 ENU/经纬度 CSV 和路径叠加图。 |
