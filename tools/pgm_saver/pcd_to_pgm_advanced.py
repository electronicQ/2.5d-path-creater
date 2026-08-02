#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np
import yaml
import os
import math

# ==========================================
# 1. 核心参数配置区
# ==========================================
file_directory = "/home/cclg/A2B_Project/A2B_ws/src/FAST_LIO/PCD"
file_name = "scansout"
pcd_format = ".pcd"
map_pgm_name = "map.pgm"
map_yaml_name = "map.yaml"

map_resolution = 0.1

# --- 三维独立直通滤波开关与阈值 ---
filter_x_en = True
thre_x_min = 15.0   
thre_x_max = 19.6

filter_y_en = False
thre_y_min = -20.0
thre_y_max = 20.0

filter_z_en = False
thre_z_min = -20.0
thre_z_max = 20.0

# --- 🌟 性能与算法控制开关 ---
use_voxel_downsample = False   # 是否使用体素降采样 (极大加速)
use_radius_filter = False     # 是否使用半径滤波去噪 (较耗时)

# 半径滤波参数 (仅在 use_radius_filter = True 时有效)
thre_radius = 0.2
thres_point_count = 5

# --- 坐标轴重映射与符号控制 ---
map_x_source = 1       # 将 PCD 的 Y 轴映射为 2D 地图的 X 轴
map_x_sign = 1         # 反向设为 -1

map_y_source = 2       # 将 PCD 的 Z 轴映射为 2D 地图的 Y 轴
map_y_sign = 1         # 反向设为 -1

# ==========================================
# 2. 核心处理逻辑
# ==========================================
def main():
    pcd_path = os.path.join(file_directory, file_name + pcd_format)
    print(f"🚀 正在加载点云文件: {pcd_path}")
    
    if not os.path.exists(pcd_path):
        print("❌ 文件不存在，请检查路径！")
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)
    
    if len(pts) == 0:
        print("❌ 点云为空！")
        return

    # --- 输出源点云极值 ---
    print("\n" + "="*40)
    print("📊 源点云物理边界范围:")
    print(f"   X 轴: [{np.min(pts[:, 0]):.4f},  {np.max(pts[:, 0]):.4f}]")
    print(f"   Y 轴: [{np.min(pts[:, 1]):.4f},  {np.max(pts[:, 1]):.4f}]")
    print(f"   Z 轴: [{np.min(pts[:, 2]):.4f},  {np.max(pts[:, 2]):.4f}]")
    print("="*40 + "\n")

    # --- 直通滤波 ---
    print("⚙️ 正在执行直通滤波...")
    if filter_x_en:
        pts = pts[(pts[:, 0] >= thre_x_min) & (pts[:, 0] <= thre_x_max)]
    if filter_y_en:
        pts = pts[(pts[:, 1] >= thre_y_min) & (pts[:, 1] <= thre_y_max)]
    if filter_z_en:
        pts = pts[(pts[:, 2] >= thre_z_min) & (pts[:, 2] <= thre_z_max)]
    
    print(f"✔️ 直通滤波后剩余点数: {len(pts)}")
    
    if len(pts) == 0:
        print("❌ 过滤过头了，点云为空！请检查直通滤波阈值。")
        return

    current_pts = pts # 用于向下传递的数据流游标

    # --- 体素降采样开关 ---
    if use_voxel_downsample:
        print("⚙️ 正在执行体素降采样...")
        pcd_temp = o3d.geometry.PointCloud()
        pcd_temp.points = o3d.utility.Vector3dVector(current_pts)
        
        # 降采样分辨率取地图分辨率即可
        pcd_down = pcd_temp.voxel_down_sample(voxel_size=map_resolution)
        current_pts = np.asarray(pcd_down.points)
        print(f"✔️ 体素降采样后剩余点数: {len(current_pts)}")

    # --- 半径滤波开关 ---
    if use_radius_filter:
        print("⚙️ 正在执行半径滤波去噪...")
        pcd_temp = o3d.geometry.PointCloud()
        pcd_temp.points = o3d.utility.Vector3dVector(current_pts) # 保证使用当前最新数据
        
        pcd_clean, ind = pcd_temp.remove_radius_outlier(nb_points=thres_point_count, radius=thre_radius)
        current_pts = np.asarray(pcd_clean.points)
        print(f"✔️ 半径滤波后剩余点数: {len(current_pts)}")

    if len(current_pts) == 0:
        print("❌ 经过降采样/滤波后，点云被清空！")
        return

    # --- 投影为 2D 占据栅格 ---
    print("🗺️ 正在生成占据栅格地图...")
    
    map_x = current_pts[:, map_x_source] * map_x_sign
    map_y = current_pts[:, map_y_source] * map_y_sign

    min_x, max_x = np.min(map_x), np.max(map_x)
    min_y, max_y = np.min(map_y), np.max(map_y)

    width = math.ceil((max_x - min_x) / map_resolution)
    height = math.ceil((max_y - min_y) / map_resolution)

    print(f"📍 2D 边界: X[{min_x:.2f}, {max_x:.2f}], Y[{min_y:.2f}, {max_y:.2f}]")
    print(f"📏 像素尺寸: {width} x {height}")

    grid = np.full((height, width), 127, dtype=np.uint8)

    idx_x = ((map_x - min_x) / map_resolution).astype(int)
    idx_y = ((map_y - min_y) / map_resolution).astype(int)

    idx_x = np.clip(idx_x, 0, width - 1)
    idx_y = np.clip(idx_y, 0, height - 1)

    grid[idx_y, idx_x] = 0 
    
    img_to_save = np.flipud(grid)

    # --- 保存文件 ---
    pgm_path = os.path.join(file_directory, map_pgm_name)
    yaml_path = os.path.join(file_directory, map_yaml_name)

    with open(pgm_path, 'wb') as f:
        f.write(b"P5\n")
        f.write(f"{width} {height}\n".encode())
        f.write(b"255\n")
        f.write(img_to_save.tobytes())

    yaml_content = {
        'image': map_pgm_name,
        'resolution': map_resolution,
        'origin': [float(min_x), float(min_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)

    print(f"✅ 成功生成传统栅格地图！")

if __name__ == "__main__":
    main()
