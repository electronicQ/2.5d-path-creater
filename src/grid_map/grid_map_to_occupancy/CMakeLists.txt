cmake_minimum_required(VERSION 3.5.1)
project(grid_map_to_occupancy)  # 包名可自定义，建议语义化

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)  # 开启编译警告，提升代码健壮性
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)          # 生成编译命令文件，方便IDE索引

# 查找ROS依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_ros       # grid_map ↔ ROS消息转换核心库
  grid_map_core      # grid_map核心数据结构
  grid_map_msgs      # grid_map自定义ROS消息
  nav_msgs           # OccupancyGrid消息依赖
  std_msgs           # 基础消息类型
  geometry_msgs      # 几何消息（grid_map坐标系依赖）
)

# 声明catkin包配置（供其他包依赖）
catkin_package(
  CATKIN_DEPENDS
    roscpp
    grid_map_ros
    grid_map_core
    grid_map_msgs
    nav_msgs
    std_msgs
    geometry_msgs
  # DEPENDS: 若有系统级依赖（如Eigen），需在此声明，本例无需
)

# 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}  # ROS依赖包的头文件目录
)

# 编译节点可执行文件
add_executable(grid_map_to_occupancy_node
  src/grid_map_to_occupancy.cpp  # 核心节点源码文件
)

# 链接依赖库（必须链接catkin_LIBRARIES，包含grid_map_ros等库）
target_link_libraries(grid_map_to_occupancy_node
  ${catkin_LIBRARIES}
)

# 确保依赖的消息生成完成后再编译本节点
add_dependencies(grid_map_to_occupancy_node
  ${catkin_EXPORTED_TARGETS}
)

# 安装可执行文件到ROS系统目录（可选，但建议添加，方便rosrun调用）
install(TARGETS grid_map_to_occupancy_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件（本例无自定义头文件，可注释）
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )
