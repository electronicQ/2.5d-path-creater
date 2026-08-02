/*
 * helpers.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/helpers.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <sys/stat.h>
#include <dirent.h>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ros/console.h>
#include <ros/package.h>

#include "grid_map_pcl/GridMapPclLoader.hpp"

namespace grid_map {
namespace grid_map_pcl {
namespace {

std::string getProjectMapPcdPath() {
  return ros::package::getPath("grid_map_pcl") + "/../../../map/pcd";
}

std::string joinPath(const std::string& folder, const std::string& filename) {
  if (folder.empty()) {
    return filename;
  }
  if (folder.back() == '/') {
    return folder + filename;
  }
  return folder + "/" + filename;
}

bool isAbsolutePath(const std::string& path) {
  return !path.empty() && path.front() == '/';
}

bool isAutoPcdFilename(const std::string& filename) {
  return filename.empty() || filename == "auto" || filename == "AUTO";
}

bool hasPcdExtension(const std::string& filename) {
  const std::string extension = ".pcd";
  if (filename.size() < extension.size()) {
    return false;
  }
  std::string suffix = filename.substr(filename.size() - extension.size());
  std::transform(suffix.begin(), suffix.end(), suffix.begin(), [](unsigned char c) { return std::tolower(c); });
  return suffix == extension;
}

bool isRegularFile(const std::string& path, time_t& modifiedTime) {
  struct stat status {};
  if (stat(path.c_str(), &status) != 0) {
    return false;
  }
  if (!S_ISREG(status.st_mode)) {
    return false;
  }
  modifiedTime = status.st_mtime;
  return true;
}

std::string findNewestPcdFile(const std::string& folderPath) {
  DIR* directory = opendir(folderPath.c_str());
  if (directory == nullptr) {
    throw std::runtime_error("Could not open PCD folder '" + folderPath + "': " + std::strerror(errno));
  }

  std::vector<std::pair<time_t, std::string>> candidates;
  while (dirent* entry = readdir(directory)) {
    const std::string filename(entry->d_name);
    if (!hasPcdExtension(filename)) {
      continue;
    }
    const std::string fullPath = joinPath(folderPath, filename);
    time_t modifiedTime {};
    if (isRegularFile(fullPath, modifiedTime)) {
      candidates.emplace_back(modifiedTime, filename);
    }
  }
  closedir(directory);

  if (candidates.empty()) {
    throw std::runtime_error("No .pcd files found in PCD folder '" + folderPath + "'.");
  }

  std::sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right) {
    if (left.first != right.first) {
      return left.first > right.first;
    }
    return left.second < right.second;
  });

  if (candidates.size() > 1) {
    ROS_WARN_STREAM("Multiple .pcd files found in " << folderPath << ". Using newest file: " << candidates.front().second);
  }

  return candidates.front().second;
}

}  // namespace

void setVerbosityLevelToDebugIfFlagSet(const ros::NodeHandle& nh) {
  bool isSetVerbosityLevelToDebug;
  nh.param<bool>("set_verbosity_to_debug", isSetVerbosityLevelToDebug, false);

  if (!isSetVerbosityLevelToDebug) {
    return;
  }

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
}

std::string getParameterPath(const ros::NodeHandle& nh) {
  std::string defaultPath = ros::package::getPath("grid_map_pcl") + "/config/parameters.yaml";
  std::string pathToConfig;
  nh.param<std::string>("config_file_path", pathToConfig, defaultPath);
  return pathToConfig;
}

std::string getOutputBagPath(const ros::NodeHandle& nh) {
  std::string pathToOutputBag;
  const std::string defaultPath = getProjectMapPcdPath() + "/output_grid_map.bag";
  nh.param<std::string>("output_grid_map", pathToOutputBag, defaultPath);
  return pathToOutputBag;
}

std::string getPcdFilePath(const ros::NodeHandle& nh) {
  std::string pathToCloud, folderPath;
  const std::string defaultPathToCloud = "auto";
  const std::string defaultFolderPath = getProjectMapPcdPath();
  nh.param<std::string>("pcd_filename", pathToCloud, defaultPathToCloud);
  nh.param<std::string>("folder_path", folderPath, defaultFolderPath);
  if (isAutoPcdFilename(pathToCloud)) {
    pathToCloud = findNewestPcdFile(folderPath);
    ROS_INFO_STREAM("Auto-selected PCD file: " << pathToCloud);
  }
  if (isAbsolutePath(pathToCloud)) {
    return pathToCloud;
  }
  return joinPath(folderPath, pathToCloud);
}

std::string getMapFrame(const ros::NodeHandle& nh) {
  std::string mapFrame;
  nh.param<std::string>("map_frame", mapFrame, "map");
  return mapFrame;
}

std::string getMapRosbagTopic(const ros::NodeHandle& nh) {
  std::string mapRosbagTopic;
  nh.param<std::string>("map_rosbag_topic", mapRosbagTopic, "grid_map");
  return mapRosbagTopic;
}

std::string getMapLayerName(const ros::NodeHandle& nh) {
  std::string mapLayerName;
  nh.param<std::string>("map_layer_name", mapLayerName, "elevation");
  return mapLayerName;
}

void saveGridMap(const grid_map::GridMap& gridMap, const ros::NodeHandle& nh, const std::string& mapTopic) {
  std::string pathToOutputBag = getOutputBagPath(nh);
  const bool savingSuccessful = grid_map::GridMapRosConverter::saveToBag(gridMap, pathToOutputBag, mapTopic);
  ROS_INFO_STREAM("Saving grid map successful: " << std::boolalpha << savingSuccessful);
}

inline void printTimeElapsedToRosInfoStream(const std::chrono::system_clock::time_point& start, const std::string& prefix) {
  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
  ROS_INFO_STREAM(prefix << duration << " sec");
}

void processPointcloud(grid_map::GridMapPclLoader* gridMapPclLoader, const ros::NodeHandle& nh) {
  const auto start = std::chrono::high_resolution_clock::now();
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  printTimeElapsedToRosInfoStream(start, "Initialization took: ");
  gridMapPclLoader->addLayerFromInputCloud(getMapLayerName(nh));
  printTimeElapsedToRosInfoStream(start, "Total time: ");
}

Eigen::Affine3f getRigidBodyTransform(const Eigen::Vector3d& translation, const Eigen::Vector3d& intrinsicRpy) {
  Eigen::Affine3f rigidBodyTransform;
  rigidBodyTransform.setIdentity();
  rigidBodyTransform.translation() << translation.x(), translation.y(), translation.z();
  Eigen::Matrix3f rotation(Eigen::Matrix3f::Identity());
  rotation *= getRotationMatrix(intrinsicRpy.x(), XYZ::X);
  rotation *= getRotationMatrix(intrinsicRpy.y(), XYZ::Y);
  rotation *= getRotationMatrix(intrinsicRpy.z(), XYZ::Z);
  rigidBodyTransform.rotate(rotation);

  return rigidBodyTransform;
}

Eigen::Matrix3f getRotationMatrix(double angle, XYZ axis) {
  Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
  switch (axis) {
    case XYZ::X: {
      rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX());
      break;
    }
    case XYZ::Y: {
      rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());
      break;
    }
    case XYZ::Z: {
      rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ());
      break;
    }
    default:
      ROS_ERROR("Unknown axis while trying to rotate the pointcloud");
  }
  return rotationMatrix;
}

Eigen::Vector3d calculateMeanOfPointPositions(Pointcloud::ConstPtr inputCloud) {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : inputCloud->points) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= inputCloud->points.size();

  return mean;
}

Pointcloud::Ptr loadPointcloudFromPcd(const std::string& filename) {
  Pointcloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

Pointcloud::Ptr transformCloud(Pointcloud::ConstPtr inputCloud, const Eigen::Affine3f& transformMatrix) {
  Pointcloud::Ptr transformedCloud(new Pointcloud());
  pcl::transformPointCloud(*inputCloud, *transformedCloud, transformMatrix);
  return transformedCloud;
}

} /* namespace grid_map_pcl*/
} /* namespace grid_map */
