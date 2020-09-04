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

OccupancyGridVisualization::OccupancyGridVisualization(const std::string & name)
: VisualizationBase(name),
  dataMin_(0.0),
  dataMax_(1.0)
{
}

OccupancyGridVisualization::~OccupancyGridVisualization()
{
}

bool OccupancyGridVisualization::readParameters()
{
  this->declare_parameter(
    std::string(this->get_name()) + ".params.layer",
    std::string("elevation"));
  this->declare_parameter(std::string(this->get_name()) + ".params.data_min", 0.0);
  this->declare_parameter(std::string(this->get_name()) + ".params.data_max", 1.0);

  if (!this->get_parameter(std::string(this->get_name()) + ".params.layer", layer_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "OccupancyGridVisualization with name '%s' did not find a 'layer' parameter.",
      this->get_name());
    return false;
  }

  if (!this->get_parameter(std::string(this->get_name()) + ".params.data_min", dataMin_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "OccupancyGridVisualization with name '%s' did not find a 'data_min' parameter.",
      this->get_name());
    return false;
  }

  if (!this->get_parameter(std::string(this->get_name()) + ".params.data_max", dataMax_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "OccupancyGridVisualization with name '%s' did not find a 'data_max' parameter.",
      this->get_name());
    return false;
  }

  return true;
}

bool OccupancyGridVisualization::initialize()
{
  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->get_name(),
    rclcpp::QoS(1).transient_local());
  return true;
}

bool OccupancyGridVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive()) {return false;}
  if (!map.exists(layer_)) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
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
