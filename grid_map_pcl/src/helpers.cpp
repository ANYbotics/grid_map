/*
 * helpers.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>

#include <cstdlib>
#include <memory>
#include <string>

#include "grid_map_pcl/helpers.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"

namespace grid_map
{
namespace grid_map_pcl
{


void setVerbosityLevelToDebugIfFlagSet(rclcpp::Node::SharedPtr & node)
{
  bool isSetVerbosityLevelToDebug;
  node->declare_parameter("set_verbosity_to_debug", false);
  node->get_parameter("set_verbosity_to_debug", isSetVerbosityLevelToDebug);

  if (!isSetVerbosityLevelToDebug) {
    return;
  }

  auto ret = rcutils_logging_set_logger_level(
    node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      node->get_logger(), "Failed to change logging severity: %s",
      rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

std::string getParameterPath()
{
  std::string filePath = ament_index_cpp::get_package_share_directory("grid_map_pcl") +
    "/config/parameters.yaml";
  return filePath;
}

std::string getOutputBagPath(rclcpp::Node::SharedPtr & node)
{
  if (!node->has_parameter("folder_path")) {
    node->declare_parameter("folder_path", std::string(""));
  }
  node->declare_parameter("output_grid_map", "output_grid_map.bag");

  std::string outputRosbagName, folderPath;
  node->get_parameter("folder_path", folderPath);
  node->get_parameter("output_grid_map", outputRosbagName);
  std::string pathToOutputBag = folderPath + "/" + outputRosbagName;
  return pathToOutputBag;
}

std::string getPcdFilePath(rclcpp::Node::SharedPtr & node)
{
  if (!node->has_parameter("folder_path")) {
    node->declare_parameter("folder_path", std::string(""));
  }
  node->declare_parameter("pcd_filename", "input_cloud");

  std::string inputCloudName, folderPath;
  node->get_parameter("folder_path", folderPath);
  node->get_parameter("pcd_filename", inputCloudName);
  std::string pathToCloud = folderPath + "/" + inputCloudName;
  return pathToCloud;
}

std::string getMapFrame(rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter("map_frame", std::string("map"));

  std::string mapFrame;
  node->get_parameter("map_frame", mapFrame);
  return mapFrame;
}

std::string getMapRosbagTopic(rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter("map_rosbag_topic", std::string("grid_map"));

  std::string mapRosbagTopic;
  node->get_parameter("map_rosbag_topic", mapRosbagTopic);
  return mapRosbagTopic;
}

std::string getMapLayerName(rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter("map_layer_name", std::string("elevation"));

  std::string mapLayerName;
  node->get_parameter("map_layer_name", mapLayerName);
  return mapLayerName;
}

void saveGridMap(
  const grid_map::GridMap & gridMap, rclcpp::Node::SharedPtr & node,
  const std::string & mapTopic)
{
  std::string pathToOutputBag = getOutputBagPath(node);
  const bool savingSuccessful = grid_map::GridMapRosConverter::saveToBag(
    gridMap, pathToOutputBag,
    mapTopic);
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Saving grid map successful: " << std::boolalpha << savingSuccessful);
}

void processPointcloud(
  grid_map::GridMapPclLoader * gridMapPclLoader,
  rclcpp::Node::SharedPtr & node)
{
  const auto start = std::chrono::high_resolution_clock::now();
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  printTimeElapsedToRosInfoStream(
    start, "Initialization took: ", node->get_logger());
  gridMapPclLoader->addLayerFromInputCloud(getMapLayerName(node));
  printTimeElapsedToRosInfoStream(start, "Total time: ", node->get_logger());
}

Eigen::Affine3f getRigidBodyTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & intrinsicRpy,
  const rclcpp::Logger & node_logger)
{
  Eigen::Affine3f rigidBodyTransform;
  rigidBodyTransform.setIdentity();
  rigidBodyTransform.translation() << translation.x(), translation.y(), translation.z();
  Eigen::Matrix3f rotation(Eigen::Matrix3f::Identity());
  rotation *= getRotationMatrix(intrinsicRpy.x(), XYZ::X, node_logger);
  rotation *= getRotationMatrix(intrinsicRpy.y(), XYZ::Y, node_logger);
  rotation *= getRotationMatrix(intrinsicRpy.z(), XYZ::Z, node_logger);
  rigidBodyTransform.rotate(rotation);

  return rigidBodyTransform;
}

Eigen::Matrix3f getRotationMatrix(
  double angle, XYZ axis,
  const rclcpp::Logger & node_logger)
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
      RCLCPP_ERROR(node_logger, "Unknown axis while trying to rotate the pointcloud");
  }
  return rotationMatrix;
}

Eigen::Vector3d calculateMeanOfPointPositions(Pointcloud::ConstPtr inputCloud)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & point : inputCloud->points) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= inputCloud->points.size();

  return mean;
}

Pointcloud::Ptr loadPointcloudFromPcd(const std::string & filename)
{
  Pointcloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

Pointcloud::Ptr transformCloud(
  Pointcloud::ConstPtr inputCloud,
  const Eigen::Affine3f & transformMatrix)
{
  Pointcloud::Ptr transformedCloud(new Pointcloud());
  pcl::transformPointCloud(*inputCloud, *transformedCloud, transformMatrix);
  return transformedCloud;
}


}  // namespace grid_map_pcl
}  // namespace grid_map
