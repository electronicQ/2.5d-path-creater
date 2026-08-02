import open3d as o3d
import numpy as np

def process_point_cloud(input_pcd_path, output_pcd_path, 
                        x_range, y_range, z_range,
                        filter_x=True, filter_y=True, filter_z=True):
    """
    读取PCD文件，输出最大最小值，并带有xyz开关的三轴直通滤波
    
    参数:
        input_pcd_path: 输入.pcd文件路径
        output_pcd_path: 输出.pcd文件路径
        x_range: x轴保留范围 (min_x, max_x)
        y_range: y轴保留范围 (min_y, max_y)
        z_range: z轴保留范围 (min_z, max_z)
        filter_x: 是否对x轴进行滤波截断
        filter_y: 是否对y轴进行滤波截断
        filter_z: 是否对z轴进行滤波截断
    """
    print(f"正在读取点云文件: {input_pcd_path} ...")
    pcd = o3d.io.read_point_cloud(input_pcd_path)
    
    if pcd.is_empty():
        print("错误：点云为空或读取失败，请检查文件路径！")
        return

    # 1. 提取所有点的坐标并计算最大/最小值
    points = np.asarray(pcd.points)
    min_bounds = points.min(axis=0)
    max_bounds = points.max(axis=0)
    
    print("\n--- 原始点云边界信息 ---")
    print(f"X轴范围: 最小 = {min_bounds[0]:.4f}, 最大 = {max_bounds[0]:.4f}")
    print(f"Y轴范围: 最小 = {min_bounds[1]:.4f}, 最大 = {max_bounds[1]:.4f}")
    print(f"Z轴范围: 最小 = {min_bounds[2]:.4f}, 最大 = {max_bounds[2]:.4f}")
    print(f"原始点云总点数: {len(points)}")

    # 2. 根据开关决定包围盒的边界
    # 如果开启滤波，则使用设定的范围；如果关闭滤波，则使用点云本身的极限边界
    min_x = x_range[0] if filter_x else min_bounds[0]
    max_x = x_range[1] if filter_x else max_bounds[0]
    
    min_y = y_range[0] if filter_y else min_bounds[1]
    max_y = y_range[1] if filter_y else max_bounds[1]
    
    min_z = z_range[0] if filter_z else min_bounds[2]
    max_z = z_range[1] if filter_z else max_bounds[2]

    print("\n--- 实际应用的滤波边界 ---")
    print(f"X轴截断范围: [{min_x:.4f}, {max_x:.4f}] (开关状态: {'开' if filter_x else '关'})")
    print(f"Y轴截断范围: [{min_y:.4f}, {max_y:.4f}] (开关状态: {'开' if filter_y else '关'})")
    print(f"Z轴截断范围: [{min_z:.4f}, {max_z:.4f}] (开关状态: {'开' if filter_z else '关'})")

    # 3. 构建三通道直通滤波器 (使用轴侧包围盒)
    min_bound = np.array([min_x, min_y, min_z], dtype=np.float64)
    max_bound = np.array([max_x, max_y, max_z], dtype=np.float64)
    
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
    
    # 执行裁剪
    filtered_pcd = pcd.crop(bbox)
    
    # 4. 输出滤波后的结果
    filtered_points = np.asarray(filtered_pcd.points)
    print("\n--- 滤波后点云信息 ---")
    print(f"滤波后点云总点数: {len(filtered_points)}")
    
    # 保存为新的.pcd文件
    o3d.io.write_point_cloud(output_pcd_path, filtered_pcd)
    print(f"\n处理完成！滤波后的点云已保存至: {output_pcd_path}")

if __name__ == "__main__":
    # 配置输入输出路径
    INPUT_FILE = "/home/cclg/A2B_Project/A2B_ws/src/FAST_LIO/PCD/scans.pcd"   # 替换为你的输入文件路径
    OUTPUT_FILE = "/home/cclg/A2B_Project/A2B_ws/src/FAST_LIO/PCD/scansout.pcd" # 替换为你的输出文件路径
    
    # 设定直通滤波的数值范围 (min, max)
    X_LIMITS = (13.5, 21.8330)
    Y_LIMITS = (0.0, 10.0)
    Z_LIMITS = (0.0, 20.0)
    
    # 【开关配置区】: True 表示执行该轴的截断，False 表示保留该轴所有点
    FILTER_X_ON = True
    FILTER_Y_ON = False  # 例如：关闭Y轴滤波，Y轴上所有点都将被保留
    FILTER_Z_ON = False
    
    process_point_cloud(
        input_pcd_path=INPUT_FILE,
        output_pcd_path=OUTPUT_FILE,
        x_range=X_LIMITS,
        y_range=Y_LIMITS,
        z_range=Z_LIMITS,
        filter_x=FILTER_X_ON,
        filter_y=FILTER_Y_ON,
        filter_z=FILTER_Z_ON
    )
