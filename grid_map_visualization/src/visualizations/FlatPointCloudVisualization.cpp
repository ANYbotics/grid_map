/*
 * FlatPointCloudVisualization.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

#include "grid_map_visualization/visualizations/FlatPointCloudVisualization.hpp"

namespace grid_map_visualization
{

FlatPointCloudVisualization::FlatPointCloudVisualization(
  const std::string & name,
  rclcpp::Node::SharedPtr nodePtr)
: VisualizationBase(name, nodePtr),
  height_(0.0)
{
}

FlatPointCloudVisualization::~FlatPointCloudVisualization()
{
}

bool FlatPointCloudVisualization::readParameters()
{
  height_ = 0.0;
  nodePtr_->declare_parameter(name_ + ".params.height", 0.0);
  if (!nodePtr_->get_parameter(name_ + ".params.height", height_)) {
    RCLCPP_INFO(
      nodePtr_->get_logger(),
      "FlatPointCloudVisualization with name '%s' "
      "did not find a 'height' parameter. Using default.", name_.c_str());
  }

  return true;
}

bool FlatPointCloudVisualization::initialize()
{
  publisher_ = nodePtr_->create_publisher<sensor_msgs::msg::PointCloud2>(
    name_,
    rclcpp::QoS(1).transient_local());
  return true;
}

bool FlatPointCloudVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive()) {return false;}
  sensor_msgs::msg::PointCloud2 pointCloud;

  grid_map::GridMap mapCopy(map);
  mapCopy.add("flat", height_);
  grid_map::GridMapRosConverter::toPointCloud(mapCopy, "flat", pointCloud);

  publisher_->publish(pointCloud);
  return true;
}

}  // namespace grid_map_visualization
