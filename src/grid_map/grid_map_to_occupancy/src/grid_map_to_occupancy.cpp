#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp> // grid_map_ros 核心头文件
#include <grid_map_core/GridMap.hpp>             // grid_map 核心数据结构
#include <nav_msgs/OccupancyGrid.h>              // OccupancyGrid 消息
#include <grid_map_msgs/GridMap.h>               // grid_map ROS 消息

// 全局发布器（发布 OccupancyGrid）
ros::Publisher occupancy_pub;

// 回调函数：处理订阅到的 grid_map 消息
void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  // 步骤1：将 ROS 格式的 grid_map 消息转为本地 grid_map 对象
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);

  // 检查目标图层（如 traversability）是否存在
  std::string target_layer = "traversability";
  if (!grid_map.exists(target_layer)) {
    ROS_WARN("grid_map 中不存在 %s 图层！", target_layer.c_str());
    return;
  }

  // 步骤2：调用 grid_map_ros 的 toOccupancyGrid 接口，转换为 OccupancyGrid
  nav_msgs::OccupancyGrid occupancy_grid;
  // 核心：traversability（0~1）→ OccupancyGrid（0~100，语义匹配）
  grid_map::GridMapRosConverter::toOccupancyGrid(
    grid_map,          // 输入：本地 grid_map 对象
    "traversability",      // 要转换的图层名（如 traversability）
    1.0,               // min：traversability=1 → OccupancyGrid=0（可通行）
    0.0,               // max：traversability=0 → OccupancyGrid=100（不可通行）
    occupancy_grid     // 输出：OccupancyGrid 消息
  );

  // 步骤3：发布 OccupancyGrid（给 move_base 导航用）
  occupancy_pub.publish(occupancy_grid);
  ROS_INFO_THROTTLE(1.0, "已发布 OccupancyGrid，栅格数：%d×%d",
                    occupancy_grid.info.width, occupancy_grid.info.height);
}

int main(int argc, char** argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "grid_map_to_occupancy_node");
  ros::NodeHandle nh;

  // 订阅 grid_map 话题（如 filters_demo 发布的 /filtered_map）
  ros::Subscriber grid_map_sub = nh.subscribe(
    "/filtered_map", 1, gridMapCallback
  );

  // 发布 OccupancyGrid 话题（move_base 通常订阅 /static_map 或 /map）
  occupancy_pub = nh.advertise<nav_msgs::OccupancyGrid>(
    "/static_map", 1, true
  );

  // 循环等待回调
  ros::spin();
  return 0;
}
