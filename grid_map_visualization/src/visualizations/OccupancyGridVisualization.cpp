/*
 * OccupancyGridVisualization.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/visualizations/OccupancyGridVisualization.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>

namespace grid_map_visualization
{

OccupancyGridVisualization::OccupancyGridVisualization(
  rclcpp::Node::SharedPtr nodeHandle,
  const std::string & name)
: VisualizationBase(nodeHandle, name),
  dataMin_(0.0),
  dataMax_(1.0)
{
}

OccupancyGridVisualization::~OccupancyGridVisualization()
{
}

bool OccupancyGridVisualization::readParameters()
{
  nodeHandle_->declare_parameter(name_ + ".params.layer", std::string("elevation"));
  nodeHandle_->declare_parameter(name_ + ".params.data_min", 0.0);
  nodeHandle_->declare_parameter(name_ + ".params.data_max", 1.0);

  if (!nodeHandle_->get_parameter(name_ + ".params.layer", layer_)) {
    RCLCPP_ERROR(
      nodeHandle_->get_logger(),
      "OccupancyGridVisualization with name '%s' did not find a 'layer' parameter.",
      name_.c_str());
    return false;
  }

  if (!nodeHandle_->get_parameter(name_ + ".params.data_min", dataMin_)) {
    RCLCPP_ERROR(
      nodeHandle_->get_logger(),
      "OccupancyGridVisualization with name '%s' did not find a 'data_min' parameter.",
      name_.c_str());
    return false;
  }

  if (!nodeHandle_->get_parameter(name_ + ".params.data_max", dataMax_)) {
    RCLCPP_ERROR(
      nodeHandle_->get_logger(),
      "OccupancyGridVisualization with name '%s' did not find a 'data_max' parameter.",
      name_.c_str());
    return false;
  }

  return true;
}

bool OccupancyGridVisualization::initialize()
{
  publisher_ = nodeHandle_->create_publisher<nav_msgs::msg::OccupancyGrid>(name_, 1);
  return true;
}

bool OccupancyGridVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive(name_)) {return true;}
  if (!map.exists(layer_)) {
    RCLCPP_WARN_STREAM(
      nodeHandle_->get_logger(),
      "OccupancyGridVisualization::visualize: No grid map layer with name '" << layer_ <<
        "' found.");
    return false;
  }
  nav_msgs::msg::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, layer_, dataMin_, dataMax_, occupancyGrid);
  publisher_->publish(occupancyGrid);
  return true;
}

}  // namespace grid_map_visualization
