import cv2
import numpy as np

# 读取你的 pgm 地图，必须加上 cv2.IMREAD_GRAYSCALE 确保以单通道灰度读取
img = cv2.imread('diji.pgm', cv2.IMREAD_GRAYSCALE)

# 获取图像中所有不重复的像素值并排序
unique_values = np.unique(img)

print("这张地图中包含的独立像素值有：")
print(unique_values)
