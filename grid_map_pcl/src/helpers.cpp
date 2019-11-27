/*
 * helpers.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/helpers.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"

#include <cstdlib>
#include <memory>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ros/console.h>
#include <ros/package.h>


namespace grid_map{
namespace grid_map_pcl {


void setVerbosityLevelToDebugIfFlagSet(const ros::NodeHandle& nh) {
  bool isSetVerbosityLevelToDebug;
  nh.param<bool>("set_verbosity_to_debug", isSetVerbosityLevelToDebug, false);

  if (!isSetVerbosityLevelToDebug){
    return;
  }

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
}

std::string getParameterPath() {
  std::string filePath = ros::package::getPath("grid_map_pcl") + "/config/parameters.yaml";
  return filePath;
}

std::string getOutputBagPath(const ros::NodeHandle& nh) {
  std::string outputRosbagName, folderPath;
  nh.param<std::string>("folder_path", folderPath, "");
  nh.param<std::string>("output_grid_map", outputRosbagName, "output_grid_map.bag");
  std::string pathToOutputBag = folderPath + "/" + outputRosbagName;
  return pathToOutputBag;
}

std::string getPcdFilePath(const ros::NodeHandle& nh) {
  std::string inputCloudName, folderPath;
  nh.param<std::string>("folder_path", folderPath, "");
  nh.param<std::string>("pcd_filename", inputCloudName, "input_cloud");
  std::string pathToCloud = folderPath + "/" + inputCloudName;
  return pathToCloud;
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

Eigen::Affine3f getRigidBodyTransform(const Eigen::Vector3d &translation,
                                      const Eigen::Vector3d &intrinsicRpy)
{
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

Eigen::Matrix3f getRotationMatrix(double angle, XYZ axis)
{
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

Eigen::Vector3d calculateMeanOfPointPositions(Pointcloud::ConstPtr inputCloud)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : inputCloud->points) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= inputCloud->points.size();

  return mean;
}

Pointcloud::Ptr loadPointcloudFromPcd(const std::string& filename)
{
  Pointcloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

Pointcloud::Ptr transformCloud(Pointcloud::ConstPtr inputCloud,
                               const Eigen::Affine3f& transformMatrix)
{
  Pointcloud::Ptr transformedCloud(new Pointcloud());
  pcl::transformPointCloud(*inputCloud, *transformedCloud, transformMatrix);
  return transformedCloud;
}


} /* namespace grid_map_pcl*/
} /* namespace grid_map */

