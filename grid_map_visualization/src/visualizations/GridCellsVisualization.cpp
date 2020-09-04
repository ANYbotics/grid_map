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

GridCellsVisualization::GridCellsVisualization(const std::string & name)
: VisualizationBase(name),
  lowerThreshold_(-std::numeric_limits<float>::infinity()),
  upperThreshold_(std::numeric_limits<float>::infinity())
{
}

GridCellsVisualization::~GridCellsVisualization()
{
}

bool GridCellsVisualization::readParameters()
{
  this->declare_parameter(
    std::string(this->get_name()) + ".params.layer",
    std::string("elevation"));
  this->declare_parameter(std::string(this->get_name()) + ".params.lower_threshold", 5.0);
  this->declare_parameter(std::string(this->get_name()) + ".params.upper_threshold", -5.0);

  if (!this->get_parameter(std::string(this->get_name()) + ".params.layer", layer_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "GridCellsVisualization with name '%s' did not find a 'layer' parameter.",
      this->get_name());
    return false;
  }

  if (!this->get_parameter(
      std::string(this->get_name()) + ".params.lower_threshold",
      lowerThreshold_))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "GridCellsVisualization with name '%s' "
      "did not find a 'lower_threshold' parameter. Using negative infinity.",
      this->get_name());
  }

  if (!this->get_parameter(
      std::string(this->get_name()) + ".params.upper_threshold",
      upperThreshold_))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "GridCellsVisualization with name '%s' "
      "did not find a 'upper_threshold' parameter. Using infinity.",
      this->get_name());
  }

  return true;
}

bool GridCellsVisualization::initialize()
{
  publisher_ = this->create_publisher<nav_msgs::msg::GridCells>(
    this->get_name(),
    rclcpp::QoS(1).transient_local());
  return true;
}

bool GridCellsVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive()) {return false;}
  if (!map.exists(layer_)) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
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
