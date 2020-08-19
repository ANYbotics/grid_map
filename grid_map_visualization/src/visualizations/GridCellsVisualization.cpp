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
#include <string>
#include <limits>

namespace grid_map_visualization
{

GridCellsVisualization::GridCellsVisualization(
  rclcpp::Node::SharedPtr node,
  const std::string & name)
: VisualizationBase(node, name),
  lowerThreshold_(-std::numeric_limits<float>::infinity()),
  upperThreshold_(std::numeric_limits<float>::infinity())
{
}

GridCellsVisualization::~GridCellsVisualization()
{
}

bool GridCellsVisualization::readParameters()
{
  node_->declare_parameter(name_ + ".params.layer", std::string("elevation"));
  node_->declare_parameter(name_ + ".params.lower_threshold", 5.0);
  node_->declare_parameter(name_ + ".params.upper_threshold", -5.0);

  if (!node_->get_parameter(name_ + ".params.layer", layer_)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GridCellsVisualization with name '%s' did not find a 'layer' parameter.",
      name_.c_str());
    return false;
  }

  if (!node_->get_parameter(name_ + ".params.lower_threshold", lowerThreshold_)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "GridCellsVisualization with name '%s' "
      "did not find a 'lower_threshold' parameter. Using negative infinity.",
      name_.c_str());
  }

  if (!node_->get_parameter(name_ + ".params.upper_threshold", upperThreshold_)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "GridCellsVisualization with name '%s' "
      "did not find a 'upper_threshold' parameter. Using infinity.",
      name_.c_str());
  }

  return true;
}

bool GridCellsVisualization::initialize()
{
  publisher_ = node_->create_publisher<nav_msgs::msg::GridCells>(name_, 1);
  return true;
}

bool GridCellsVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive()) {return true;}
  if (!map.exists(layer_)) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(),
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
