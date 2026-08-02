cmake_minimum_required(VERSION 3.0.2)
project(pcd_z_filter)

# 1. 查找 catkin 依赖的包
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)

# 2. 查找独立的 PCL 库
find_package(PCL REQUIRED)

# 3. 声明 catkin 软件包的导出配置
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES pcd_z_filter
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp
  # DEPENDS system_lib
)

# 4. 指定头文件搜索路径
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 5. 声明要编译的可执行文件节点
add_executable(${PROJECT_NAME}_node src/pcd_z_filter_node.cpp)

# 6. 为可执行文件链接所需的库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)