# 项目上下文

本文档是本仓库的长期项目背景文件，用于替代根目录旧的 `CODEX_PROJECT_PROMPT.md`。后续涉及地图转换、坐标系、规划器、输出格式和未知区域处理的任务，应优先参考本文档。

## 项目目标

本项目的主流程是：

```text
已知原点经纬高的局部 ENU .pcd 点云地图
-> grid_map 生成高程/可通行栅格
-> 保存连续灰度静态 pgm + yaml 地图
-> Python 规划器读取静态地图
-> 根据起点/终点经纬度生成三条全局轨迹
-> 同时导出 ENU CSV 和经纬度 CSV
```

当前重点不是重新做点云建图，而是把已有 `.pcd`、`grid_map`、静态地图保存和 `src/` 中的离线路径规划流程串联稳定。

## 坐标系约定

- `.pcd` 点云地图内部坐标为局部 ENU，单位通常为米。
- 点云地图局部 ENU 原点的经度、纬度和高度是已知元数据。
- 用户输入的起点和终点使用经纬度。
- 规划内部必须使用 ENU 坐标和栅格坐标，不要直接用经纬度做栅格搜索。
- 最终轨迹必须输出经纬度，供后续循迹导航在不同 ENU 坐标系下转换使用。
- ENU 轨迹仍要保留，用于调试、地图对齐和结果复现。

不要混淆三类坐标：`latitude, longitude` 是地理坐标，`x, y` 是局部 ENU 米制坐标，`row, col` 是 PGM/栅格索引。

## 静态 PGM 灰度语义

静态 `.pgm` 是连续灰度代价地图，不是简单二值地图。

- `gray == 205`：未知区域。
- `gray <= absolute_obstacle_gray`：绝对障碍，当前默认阈值为 `90`。
- `gray > absolute_obstacle_gray` 且 `gray != 205`：可参与规划的连续代价栅格。
- 灰度越大越接近白色，通行代价越低；灰度越小越接近黑色，通行代价越高。
- 灰度到代价的映射必须单调，例如 `1.0 - gray / 255.0`。

当前版本中，未知区域 `205` 严格禁止通行。小洞未知区域修补是后续升级计划，不是默认行为。

## grid_map 未知区域处理

点云没有观测到的区域不能在 `grid_map` 处理链中被误当作自由空间。

必须保持以下链路：

```text
原始 elevation 没有点云观测
-> 保持 NaN
-> 最终 traversability 仍保留未知语义
-> OccupancyGrid 中导出为 -1
-> tools/pgm_saver/save_continuous_map.py 保存为 PGM 灰度 205
```

如果滤波链中需要补洞或 inpainting，只能作为中间计算辅助；最终发布或保存前必须恢复原始未知区域 mask。

## 轨迹输出约定

规划器应从同一起点到同一终点生成三条有差异的轨迹：

- 距离优先
- 平滑优先
- 综合代价/安全距离优先

每条轨迹都导出两种 CSV：

```text
traj_distance_geo.csv
traj_distance_enu.csv
traj_smooth_geo.csv
traj_smooth_enu.csv
traj_cost_aware_geo.csv
traj_cost_aware_enu.csv
```

`*_geo.csv` 用于后续循迹，`*_enu.csv` 用于调试和检查。

## 后续升级计划

后续可以增加静态 PGM 预处理：识别小范围 `205` 未知洞，将其转换为有限代价区域；但大片未知区域仍必须保持不可通行。该功能必须是显式、可配置的规划前处理步骤。
