import cv2
import numpy as np
import sys

def export_map_pixels(input_file, output_file):
    # 1. 以单通道灰度模式读取 pgm 地图
    img = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)

    if img is None:
        print(f"错误: 无法读取文件 '{input_file}'。请确保文件存在且路径正确。")
        sys.exit(1)

    height, width = img.shape
    print(f"成功加载地图 '{input_file}'")
    print(f"地图分辨率 (高 x 宽): {height} x {width} 像素")
    print(f"总计像素数量: {height * width}")

    # 2. 将二维矩阵保存到文件
    # fmt='%d' 确保保存为整数格式 (0-255)
    # delimiter=',' 使用逗号分隔，这是标准 CSV 格式，方便表格软件完美对齐读取
    print(f"正在将像素矩阵导出到 '{output_file}'...")
    
    try:
        np.savetxt(output_file, img, fmt='%d', delimiter=',')
        print("导出完成！")
        print("建议：请使用 Excel、WPS 或 Numbers 打开此 CSV 文件。")
        print("说明：表格中的第 1 行第 1 列 (A1) 对应地图左上角的像素 (x=0, y=0)。")
    except Exception as e:
        print(f"导出失败: {e}")

if __name__ == "__main__":
    # 在这里修改你的输入和输出文件名
    INPUT_PGM_FILE = 'diji.pgm'
    OUTPUT_CSV_FILE = 'map_cost_matrix.csv'
    
    export_map_pixels(INPUT_PGM_FILE, OUTPUT_CSV_FILE)
