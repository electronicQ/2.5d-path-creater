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
output/road_map1.xml
output/road_map2.xml
output/road_map3.xml
output/overlay.png
```

- `road_map1.xml`：代价感知（`cost_aware`）轨迹的局部 UTM XML。
- `road_map2.xml`：距离优先（`distance`）轨迹的局部 UTM XML。
- `road_map3.xml`：平滑优先（`smooth`）轨迹的局部 UTM XML。
- `overlay.png`：在 PGM 地图上使用同色细线连接每组轨迹点；距离优先为红色圆点，平滑优先为绿色三角形，综合代价优先为蓝色正方形。分层尺寸保证重叠点仍能显示三种颜色，并保留起点和终点外圈标记。

局部 UTM 坐标的定义为：

```text
local_x = UTM_easting(point) - UTM_easting(ENU_origin)
local_y = UTM_northing(point) - UTM_northing(ENU_origin)
```

UTM 分区和南北半球根据 ENU 原点选择，XML 中的 `x`、`y` 单位为米，`yaw` 单位为弧度。XML 是“以 ENU 原点为原点的局部 UTM 偏移坐标”，与 ENU 使用相同的原点，但不是严格意义上的 ENU 坐标。文件名映射固定为：`road_map1.xml = road_map_cost_aware.xml`、`road_map2.xml = road_map_distance.xml`、`road_map3.xml = road_map_smooth.xml`。XML 只包含 `<map>` 根节点和 `<vertex>` 轨迹点。

默认会对 A* 输出先做直线可视化 shortcut，再做安全圆角曲线平滑，然后按弦误差、航向变化和最大间距自适应采样。长直线上的点距最大约 10 m，缓弯约 1～2 m，急弯约 0.3～0.8 m；XML 保存最终轨迹点。

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
tracking_sampling:
  enabled: true
  max_spacing_m: 10.0
  min_spacing_m: 0.3
  chord_tolerance_m: 0.15
  max_heading_change_deg: 3.0
corridor_centering:
  enabled: true
  apply_to: cost_aware
  high_cost_gray_threshold: 220
  target_clearance_m: 3.0
  weight: 20.0
  shortcut_min_clearance_m: 2.5
  curve_min_clearance_m: 2.0
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
+ w_center * corridor_deficit²  # 仅 corridor_centering.apply_to 指定策略
```

三组默认策略：

- `distance`：更偏短路径
- `smooth`：更重视少转弯和平滑
- `cost_aware`：更重视高灰度区域和障碍物安全距离，并通过中心走廊代价减少弯道内侧贴边

如果三条路径太像，可以拉大各策略权重差异，尤其是 `w_turn`、`w_clear` 和 `w_gray`。

`smooth_paths: true` 会在规划完成后做 line-of-sight shortcut smoothing：如果两个非相邻路径点之间的直线段全程处在膨胀后的自由栅格内，就移除中间冗余点。这个步骤不会穿越未知区、障碍区或障碍膨胀区，用来减少栅格搜索带来的锯齿折线。

`curve_smoothing.enabled: true` 会继续对 shortcut 后的路径做安全圆角化：每个折角会尝试用二次 Bezier 曲线替代，曲线采样点和相邻采样点连线都必须处在膨胀后的自由栅格内。

`tracking_sampling` 决定最终 XML 轨迹点密度。采样时向前寻找尽可能远的新点，同时限制最大点距、曲线弦误差和累计航向变化。该方式不直接依赖栅格曲率，因此能稳定表现为直线稀疏、缓弯适中、急弯密集。`min_spacing_m` 是最小点距，`max_spacing_m` 是直线最大点距。

`corridor_centering` 只作用于 `apply_to` 指定的策略，默认是 `cost_aware`，因此只影响 `road_map1.xml`。规划器先把灰度小于等于 `high_cost_gray_threshold` 的区域视为道路两侧高代价边界，再计算每个自由格到最近边界的中心距离。A* 对低于 `target_clearance_m` 的格子施加平方中心偏离代价；`shortcut_min_clearance_m` 和 `curve_min_clearance_m` 进一步防止快捷平滑或圆角把轨迹带回弯道内侧。

中心线阈值必须结合实际 PGM 调整：阈值太高会把正常道路也当成边界，阈值太低则可能忽略灰色高代价区域。距离优先和平滑优先不受该配置影响。

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
| `curve_smoothing.py` | 对 shortcut 路径做安全圆角平滑，并按几何误差自适应采样巡迹点。 |
| `geo.py` | WGS84、ECEF、ENU 坐标转换。 |
| `map_metadata.py` | 读取地图 ENU 原点元数据。 |
| `auto_endpoints.py` | 解析起终点并检查可通行性。 |
| `trajectory.py` | 写出三份轨迹 XML 和离散轨迹点叠加图。 |
