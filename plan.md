# Fast-LIO2 地图后处理对齐到局部 ENU 的备用方案

当前仍优先使用 LIGO 的 NMEA 融合建图流程。本文件只记录一个备用后处理方案，暂不实行。

## 目标

使用 Fast-LIO2 生成几何更稳定的局部点云地图，再利用 RTK/NMEA 转出的 ENU 轨迹，离线求解从 Fast-LIO2 局部地图系到局部 ENU 坐标系的刚体变换。

最终得到：

```text
p_enu = R * p_lio + t
```

其中：

- `p_lio`：Fast-LIO2 输出的局部地图坐标。
- `p_enu`：GNSS/RTK 转换后的局部 ENU 坐标。
- `R`：从 LIO 局部坐标系到 ENU 坐标系的旋转，优先只估计 yaw。
- `t`：从 LIO 局部坐标系到 ENU 坐标系的平移。

不估计尺度 `scale`，因为 LiDAR/IMU 和 ENU 都使用米作为单位。

## 推荐对齐模型

优先使用 SE(2) 对齐：

```text
x_enu, y_enu = yaw_rotation(x_lio, y_lio) + t_xy
z_enu = z_lio + t_z
```

不建议第一版直接使用完整 SE(3) 对齐，原因是：

- RTK 高度通常比平面位置更噪。
- LiDAR-IMU 的 roll 和 pitch 已经由 IMU 重力约束。
- 用噪声较大的 RTK 高度参与三维姿态估计，可能把地图整体倾斜。

## 输入数据

需要同时保存两条轨迹：

1. Fast-LIO2 里程计轨迹：

```text
timestamp, x_lio, y_lio, z_lio
```

2. RTK/NMEA 转 ENU 后的轨迹：

```text
timestamp, x_enu, y_enu, z_enu
```

当前工程中 ENU 轨迹可以继续来自： 

```text
/rtk/gps_odom
```

该话题由 `gnss_to_enu` 生成。

## 处理流程

1. 使用 Fast-LIO2 正常建图，保存局部坐标系下的 `.pcd` 地图。

2. 同时导出 Fast-LIO2 odometry 和 `/rtk/gps_odom`。

3. 按时间戳配对轨迹点。

   对每一个 RTK 时间戳 `t_i`，在 Fast-LIO2 odometry 中插值得到同一时刻的 LIO 位姿：

   ```text
   p_lio(t_i) <-> p_enu(t_i)
   ```

4. 过滤低质量点对。

   建议过滤：

   - 时间差过大的点对。
   - 机器人静止或速度过低的点。
   - RTK 明显跳变的点。
   - 残差明显偏大的点。
   - 如果能读取定位状态，则过滤质量过差的 GNSS 点。

5. 用二维 SVD 求解 `yaw + x/y 平移`。

   对点集：

   ```text
   P = { [x_lio, y_lio] }
   Q = { [x_enu, y_enu] }
   ```

   计算：

   ```text
   p_mean = mean(P)
   q_mean = mean(Q)

   P0 = P - p_mean
   Q0 = Q - q_mean

   H = P0^T * Q0
   U, S, Vt = svd(H)
   R = V * U^T
   t = q_mean - R * p_mean
   ```

   如果 `det(R) < 0`，需要修正反射。

6. 单独估计高度平移：

```text
t_z = median(z_enu - z_lio)
```

7. 将 Fast-LIO2 输出的 `.pcd` 地图整体变换到 ENU 坐标系。

   对每个点执行：

```text
[x_enu, y_enu]^T = R_2d * [x_lio, y_lio]^T + t_xy
z_enu = z_lio + t_z
```

8. 保存新的 ENU `.pcd` 地图。

9. 后续继续使用当前工程已有流程：

```bash
roslaunch grid_map_pcl grid_map_pcl_loader_node.launch
roslaunch grid_map_demos filters_demo_gai.launch
python3 tools/pgm_saver/save_continuous_map.py -n banfu2
python3 -m tools.path_creater.main
```

## 方案优势

- Fast-LIO2 负责生成局部几何质量较好的点云地图。
- RTK/NMEA 只负责确定全局 ENU 坐标系，不直接参与每一帧建图。
- 浮点解 RTK 的局部噪声不会直接把地图拉变形。
- 后处理可加入 RANSAC、残差剔除和人工检查，比在线融合更可控。

## 适用条件

该方案适合 Fast-LIO2 轨迹本身局部稳定、累计漂移较小的情况。

如果 Fast-LIO2 轨迹已经明显弯曲或累计漂移严重，单个全局 `R, t` 只能完成整体坐标系对齐，不能修正中间的非刚性漂移。那种情况需要进一步考虑分段对齐或 pose graph 优化。

## 当前决策

暂不实行该后处理方案。

当前仍继续使用 LIGO 的 NMEA 融合建图能力，将地图直接构建在局部 ENU 坐标系下。
