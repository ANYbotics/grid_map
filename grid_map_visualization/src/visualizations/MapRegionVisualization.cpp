/*
 * MapRegionVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/visualizations/MapRegionVisualization.hpp"
#include <grid_map_visualization/GridMapVisualizationHelpers.hpp>

// ROS
#include <geometry_msgs/msg/point.hpp>

#include <string>

using namespace std::chrono_literals;

namespace grid_map_visualization
{

MapRegionVisualization::MapRegionVisualization(
  const std::string & name,
  rclcpp::Node::SharedPtr nodePtr)
: VisualizationBase(name, nodePtr),
  nVertices_(5),
  lineWidth_(0.01)
{
}

MapRegionVisualization::~MapRegionVisualization()
{
}

bool MapRegionVisualization::readParameters()
{
  nodePtr_->declare_parameter(name_ + ".params.line_width", 0.003);
  nodePtr_->declare_parameter(name_ + ".params.color", 16777215);
  lineWidth_ = 0.003;
  if (!nodePtr_->get_parameter(name_ + ".params.line_width", lineWidth_)) {
    RCLCPP_INFO(
      nodePtr_->get_logger(),
      "MapRegionVisualization with name '%s' did not find a 'line_width' parameter. Using default.",
      name_.c_str());
    return false;
  }

  int colorValue = 16777215;  // white, http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D20%2C+g%3D50%2C+b%3D230%7D  // NOLINT
  if (!nodePtr_->get_parameter(name_ + ".params.color", colorValue)) {
    RCLCPP_INFO(
      nodePtr_->get_logger(),
      "MapRegionVisualization with name '%s' did not find a 'color' parameter. Using default.",
      name_.c_str());
  }
  setColorFromColorValue(color_, colorValue, true);

  return true;
}

bool MapRegionVisualization::initialize()
{
  marker_.ns = "map_region";
  marker_.lifetime = rclcpp::Duration(0ns);  // Setting lifetime to forever
  marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_.scale.x = lineWidth_;
  marker_.points.resize(nVertices_);  // Initialized to [0.0, 0.0, 0.0]
  marker_.colors.resize(nVertices_, color_);
  publisher_ = nodePtr_->create_publisher<visualization_msgs::msg::Marker>(
    name_,
    rclcpp::QoS(1).transient_local());
  return true;
}

bool MapRegionVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive()) {return false;}

  // TODO(needs_assignment): Replace this with ploygon?

  // Set marker info.
  marker_.header.frame_id = map.getFrameId();
  marker_.header.stamp = rclcpp::Time(map.getTimestamp());

  // Adapt positions of markers.
  float halfLengthX = map.getLength().x() / 2.0;
  float halfLengthY = map.getLength().y() / 2.0;

  marker_.points[0].x = map.getPosition().x() + halfLengthX;
  marker_.points[0].y = map.getPosition().y() + halfLengthY;
  marker_.points[1].x = map.getPosition().x() + halfLengthX;
  marker_.points[1].y = map.getPosition().y() - halfLengthY;
  marker_.points[2].x = map.getPosition().x() - halfLengthX;
  marker_.points[2].y = map.getPosition().y() - halfLengthY;
  marker_.points[3].x = map.getPosition().x() - halfLengthX;
  marker_.points[3].y = map.getPosition().y() + halfLengthY;
  marker_.points[4].x = marker_.points[0].x;
  marker_.points[4].y = marker_.points[0].y;

  publisher_->publish(marker_);
  return true;
}

}  // namespace grid_map_visualization
