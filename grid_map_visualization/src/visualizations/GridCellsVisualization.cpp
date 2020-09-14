/*
 * GridCellsVisualization.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/visualizations/GridCellsVisualization.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/grid_cells.hpp>

// STD
#include <limits>
#include <string>

namespace grid_map_visualization
{

GridCellsVisualization::GridCellsVisualization(
  const std::string & name,
  rclcpp::Node::SharedPtr node_ptr)
: VisualizationBase(name, node_ptr),
  lowerThreshold_(-std::numeric_limits<float>::infinity()),
  upperThreshold_(std::numeric_limits<float>::infinity())
{
}

GridCellsVisualization::~GridCellsVisualization()
{
}

bool GridCellsVisualization::readParameters()
{
  node_ptr_->declare_parameter(
    name_ + ".params.layer",
    std::string("elevation"));
  node_ptr_->declare_parameter(name_ + ".params.lower_threshold", 5.0);
  node_ptr_->declare_parameter(name_ + ".params.upper_threshold", -5.0);

  if (!node_ptr_->get_parameter(name_ + ".params.layer", layer_)) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(),
      "GridCellsVisualization with name '%s' did not find a 'layer' parameter.",
      name_);
    return false;
  }

  if (!node_ptr_->get_parameter(name_ + ".params.lower_threshold", lowerThreshold_)) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "GridCellsVisualization with name '%s' "
      "did not find a 'lower_threshold' parameter. Using negative infinity.",
      name_);
  }

  if (!node_ptr_->get_parameter(name_ + ".params.upper_threshold", upperThreshold_)) {
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "GridCellsVisualization with name '%s' "
      "did not find a 'upper_threshold' parameter. Using infinity.", name_);
  }

  return true;
}

bool GridCellsVisualization::initialize()
{
  publisher_ = node_ptr_->create_publisher<nav_msgs::msg::GridCells>(
    name_,
    rclcpp::QoS(1).transient_local());
  return true;
}

bool GridCellsVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive()) {return false;}
  if (!map.exists(layer_)) {
    RCLCPP_WARN_STREAM(
      node_ptr_->get_logger(),
      "GridCellsVisualization::visualize: No grid map layer with name '" << layer_ << "' found.");
    return false;
  }
  nav_msgs::msg::GridCells gridCells;
  grid_map::GridMapRosConverter::toGridCells(
    map, layer_, lowerThreshold_, upperThreshold_,
    gridCells);
  publisher_->publish(gridCells);
  return true;
}

}  // namespace grid_map_visualization
