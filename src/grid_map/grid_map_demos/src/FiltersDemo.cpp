/*
 * FiltersDemo.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 *
 */

#include "grid_map_demos/FiltersDemo.hpp"

using namespace grid_map;

namespace grid_map_demos {


// （1）这个是构造函数，也就是这个FiltersDemo的构造函数。
// 需要两个参数，一个是ros大管家，另一个是bool值。
// （2）这个构造函数也继承两个类：一个是大管家类，可以使用大管家类的多种方法；
// 另一个是filterChain_类，可以用来使用filterChain_类中的各种方法。
// （3）这个FiltersDemo类的作用就是：
// 一个订阅者，订阅inputTopic_这个话题。
// 另一个是发布者，发布grid_map数据到outputTopic_中。
FiltersDemo::FiltersDemo(ros::NodeHandle& nodeHandle, bool& success)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap")
{
  if (!readParameters()) {
    success = false;
    return;
  }

  subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &FiltersDemo::callback, this);
  publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(outputTopic_, 1, true);

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }

  success = true;
}

FiltersDemo::~FiltersDemo()
{
}

bool FiltersDemo::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_)) {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("output_topic", outputTopic_, std::string("output"));
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  return true;
}

void FiltersDemo::callback(const grid_map_msgs::GridMap& message)
{
  // Convert message to map.
  GridMap inputMap;
  GridMapRosConverter::fromMessage(message, inputMap);

  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }

  ROS_INFO("PUBLISH");
  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  publisher_.publish(outputMessage);
  // ================= 添加以下代码自动保存 =================
  // std::string savePath = "/home/cclg/A2B_Project/A2B_ws/src/grid_map/grid_map_filters/bag/final_map.bag"; // 改成你想要的绝对路径
  // grid_map::GridMapRosConverter::saveToBag(outputMap, savePath, outputTopic_);
  // ROS_INFO_STREAM("Successfully saved the multi-layer grid map to: " << savePath);
  // ======================================================
}

} /* namespace */
