#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <limits>

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "pcd_z_filter_node");
    ros::NodeHandle nh("~");

    std::string input_pcd_path, output_pcd_path;
    double z_max_threshold;

    // 从参数服务器获取参数，并设置默认值
    nh.param<std::string>("input_file", input_pcd_path, "input.pcd");
    nh.param<std::string>("output_file", output_pcd_path, "output.pcd");
    nh.param<double>("z_threshold", z_max_threshold, 1.0); // 默认保留 Z <= 1.0 的点云

    ROS_INFO("Input file: %s", input_pcd_path.c_str());
    ROS_INFO("Output file: %s", output_pcd_path.c_str());
    ROS_INFO("Z max threshold: %f", z_max_threshold);

    // 1. 读取原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read the input PCD file.");
        return -1;
    }
    ROS_INFO("Loaded %lu data points from input file.", cloud->points.size());

    // 2. 设置直通滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // 保留 Z 轴从最低点到设定高度 z_max_threshold 之间的点
    pass.setFilterLimits(std::numeric_limits<float>::lowest(), z_max_threshold);
    // pass.setFilterLimitsNegative(true); // 如果你想反向操作（保留高于该高度的点），可以取消注释这行
    
    // 3. 执行滤波
    pass.filter(*cloud_filtered);
    ROS_INFO("Filtered cloud contains %lu data points.", cloud_filtered->points.size());

    // 4. 保存为新的 PCD 文件
    if (!cloud_filtered->empty())
    {
        pcl::io::savePCDFileBinary(output_pcd_path, *cloud_filtered);
        ROS_INFO("Saved filtered point cloud to %s.", output_pcd_path.c_str());
    }
    else
    {
        ROS_WARN("Filtered point cloud is empty. Nothing to save.");
    }

    return 0;
}