import cv2
import numpy as np

# 读取你的 pgm 地图，必须加上 cv2.IMREAD_GRAYSCALE 确保以单通道灰度读取
# 请确保 'diji.pgm' 与此脚本在同一目录下，或提供绝对路径
img = cv2.imread('shuma.pgm', cv2.IMREAD_GRAYSCALE)

if img is None:
    print("错误：无法读取文件 'diji.pgm'。请检查文件是否存在以及路径是否正确。")
else:
    # return_counts=True 会让函数同时返回独立值数组和对应的频数数组
    unique_values, counts = np.unique(img, return_counts=True)

    print("这张地图中包含的像素值及其对应的数量统计如下：")
    print("-" * 40)
    print(f"{'像素值 (Pixel Value)':<20} | {'数量 (Count)'}")
    print("-" * 40)
    
    # 使用 zip 将像素值和对应的数量打包遍历
    for val, count in zip(unique_values, counts):
        print(f"{val:<22} | {count}")
        
    print("-" * 40)
    # 顺便输出一下地图的基础信息，方便核对
    print(f"总计独立像素值种类: {len(unique_values)} 种")
    print(f"总计像素点数量: {np.sum(counts)} (地图分辨率: {img.shape[1]}x{img.shape[0]})")
