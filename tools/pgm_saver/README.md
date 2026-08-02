# pgm_saver

`tools/pgm_saver` 用于把 ROS 中发布的连续 `nav_msgs/OccupancyGrid` 地图保存为静态 `.pgm` + `.yaml` 文件。当前主脚本是 `save_continuous_map.py`。

## 使用场景

在完整流程中，它位于 `grid_map` 可通行层之后：

```text
grid_map traversability layer
-> /grid_map_visualization/traversability_grid
-> save_continuous_map.py
-> map/pgm/*.pgm + map/pgm/*.yaml
```

也就是说，先运行点云到高程图、再运行高程图到 `traversability` 的滤波链，确认 `/grid_map_visualization/traversability_grid` 已经有数据后，再运行本脚本。

## 订阅话题

脚本固定订阅：

```text
/grid_map_visualization/traversability_grid
```

消息类型：

```text
nav_msgs/OccupancyGrid
```

该话题由 `grid_map_visualization` 根据 `traversability` 层转换发布。当前工程要求未知区域保持以下链路：

```text
elevation NaN
-> traversability NaN
-> OccupancyGrid -1
-> PGM gray 205
```

## 运行

从仓库根目录执行：

```bash
source devel/setup.bash
python3 tools/pgm_saver/save_continuous_map.py -n example_map
```

输出：

```text
map/pgm/example_map.pgm
map/pgm/example_map.yaml
```

如果不传 `-n`，默认输出：

```text
map/pgm/map.pgm
map/pgm/map.yaml
```

## 输出路径规则

`-n` / `--name` 是输出文件基名，不需要带扩展名。

| 参数示例 | 实际输出 |
| --- | --- |
| `-n example_map` | `map/pgm/example_map.pgm` 和 `map/pgm/example_map.yaml` |
| `-n map/pgm/example_map` | `map/pgm/example_map.pgm` 和 `map/pgm/example_map.yaml` |
| `-n custom/example_map` | `custom/example_map.pgm` 和 `custom/example_map.yaml` |
| `-n /tmp/example_map` | `/tmp/example_map.pgm` 和 `/tmp/example_map.yaml` |

裸文件名会自动保存到工程约定目录 `map/pgm`。显式相对路径按仓库根目录解析。绝对路径按原样使用。

## 灰度映射

`save_continuous_map.py` 保留连续占据代价，不把地图二值化。

| OccupancyGrid 值 | PGM 灰度 | 语义 |
| --- | --- | --- |
| `-1` | `205` | 未知区域 |
| `100` 或更大 | `0` | 障碍 / 最高占据 |
| `0` 或更小 | `254` | 最可通行 |
| `1..99` | `255 - value / 100 * 255` | 连续代价 |

生成的 `.yaml` 使用：

```yaml
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: scale
```

`mode: scale` 是关键点：后续读取时保留连续灰度，不按二值阈值简单切分地图。

## 常见问题

- 没有生成文件：确认 `/grid_map_visualization/traversability_grid` 正在发布。
- 输出全白：检查上游 filter chain 是否把原始 `elevation` 的 NaN 恢复到 `traversability`，否则未知区域会被当成可通行。
- PGM 没有 `205`：先用 `rostopic echo /grid_map_visualization/traversability_grid` 检查 data 中是否存在 `-1`。
- YAML 中 `image` 只写文件名：这是有意设计，保证 `.pgm` 和 `.yaml` 放在同一目录时可以整体移动。

## 相关脚本

同目录下还有一些早期检查/转换脚本，例如 `check_map.py`、`check_map_plus.py`、`check_fenbu.py`。当前主流程优先使用 `save_continuous_map.py` 保存连续静态地图。
