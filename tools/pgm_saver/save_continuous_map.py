#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import argparse
import math
import sys
try:
    from pathlib import Path
except ImportError:  # ROS Melodic / Python 2.7
    from pathlib2 import Path

try:
    import rospy
    from nav_msgs.msg import OccupancyGrid
except ImportError:
    rospy = None
    OccupancyGrid = None


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "map" / "pgm"


def _strip_map_suffix(path):
    if path.suffix.lower() in (".pgm", ".yaml", ".yml"):
        return path.with_suffix("")
    return path


def default_output_base():
    return DEFAULT_OUTPUT_DIR / "map"


def resolve_output_base(name):
    if name is None or str(name).strip() == "":
        return default_output_base()

    path = _strip_map_suffix(Path(str(name)).expanduser())
    if path.is_absolute():
        return path
    if len(path.parts) == 1:
        return DEFAULT_OUTPUT_DIR / path
    return (PROJECT_ROOT / path).resolve()


def occupancy_to_pgm_array(values, width, height):
    data = np.array(values).reshape((height, width))
    img = np.full((height, width), 255, dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            val = int(data[y, x])
            if val == -1:
                img[y, x] = 205
            elif val >= 100:
                img[y, x] = 0
            elif val <= 0:
                img[y, x] = 254
            else:
                img[y, x] = int(255 - (val / 100.0) * 255)
    return img

class ContinuousMapSaver:
    def __init__(self, output_base):
        self.output_base = resolve_output_base(output_base)
        rospy.init_node('hd_map_saver', anonymous=True)
        # 订阅你的占据栅格话题
        rospy.Subscriber('/grid_map_visualization/traversability_grid', OccupancyGrid, self.map_callback)
        rospy.loginfo("等待地图数据... 准备保存为: {0}.pgm 和 {0}.yaml".format(self.output_base))
        rospy.spin()

    def map_callback(self, msg):
        rospy.loginfo("成功拦截到地图数据！正在处理...")
        width = msg.info.width
        height = msg.info.height
        
        # --- 1. 处理图像数据 ---
        img = occupancy_to_pgm_array(msg.data, width, height)

        # 翻转图像坐标系匹配 ROS
        img = np.flipud(img)

        # --- 2. 提取原点与姿态 ---
        # 从四元数计算 Yaw 角 (绕Z轴旋转)
        q = msg.info.origin.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # --- 3. 保存 .pgm 文件 ---
        self.output_base.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = self.output_base.parent / (self.output_base.name + ".pgm")
        # Binary PGM works on both Python 2 and 3 without cv_bridge/OpenCV ABI.
        with open(str(pgm_path), "wb") as stream:
            stream.write(("P5\n{} {}\n255\n".format(width, height)).encode("ascii"))
            stream.write(img.tobytes())

        # --- 4. 生成并保存 .yaml 文件 ---
        yaml_path = self.output_base.parent / (self.output_base.name + ".yaml")
        # Quote the filename as a JSON string, which is also a valid YAML scalar.
        import json
        yaml_content = (
            "image: {}\nresolution: {}\norigin: [{}, {}, {}]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: scale\n"
        ).format(json.dumps(pgm_path.name), msg.info.resolution, origin_x, origin_y, yaw)
        with open(str(yaml_path), "w") as stream:
            stream.write(yaml_content)

        rospy.loginfo("地图已保存: {} 和 {}".format(pgm_path, yaml_path))
        rospy.signal_shutdown("Finished saving map")

if __name__ == '__main__':
    if rospy is None:
        raise RuntimeError("ROS is required to run the map saver node.")

    # 剥离 ROS 注入的额外参数，只留给 argparse 处理我们的参数
    argv = rospy.myargv(argv=sys.argv)
    
    parser = argparse.ArgumentParser(description="Save a continuous OccupancyGrid map to PGM and YAML.")
    parser.add_argument(
        '-n',
        '--name',
        type=str,
        default='map',
        help='Output base filename. Bare names are saved under map/pgm.',
    )
    args = parser.parse_args(argv[1:])
    
    # 启动保存器
    saver = ContinuousMapSaver(args.name)
