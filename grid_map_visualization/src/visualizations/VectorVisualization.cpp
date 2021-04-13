/*
 * VectorVisualization.cpp
 *
 *  Created on: Sep 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/visualizations/VectorVisualization.hpp"

// Iterator
#include <grid_map_core/iterators/GridMapIterator.hpp>

// Color conversion
#include <grid_map_visualization/GridMapVisualizationHelpers.hpp>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <string>

using namespace std::chrono_literals;

namespace grid_map_visualization
{

VectorVisualization::VectorVisualization(const std::string & name, rclcpp::Node::SharedPtr nodePtr)
: VisualizationBase(name, nodePtr)
{
}

VectorVisualization::~VectorVisualization()
{
}

bool VectorVisualization::readParameters()
{
  nodePtr_->declare_parameter(name_ + ".params.layer_prefix", std::string(""));
  nodePtr_->declare_parameter(name_ + ".params.position_layer", std::string(""));
  nodePtr_->declare_parameter(name_ + ".params.scale", 1.0);
  nodePtr_->declare_parameter(name_ + ".params.line_width", 0.003);
  nodePtr_->declare_parameter(name_ + ".params.color", 65280);

  std::string typePrefix;
  if (!nodePtr_->get_parameter(name_ + ".params.layer_prefix", typePrefix)) {
    RCLCPP_ERROR(
      nodePtr_->get_logger(),
      "VectorVisualization with name '%s' did not find a 'layer_prefix' parameter.",
      name_.c_str());
    return false;
  }
  types_.push_back(typePrefix + "x");
  types_.push_back(typePrefix + "y");
  types_.push_back(typePrefix + "z");

  if (!nodePtr_->get_parameter(
      name_ + ".params.position_layer",
      positionLayer_))
  {
    RCLCPP_ERROR(
      nodePtr_->get_logger(),
      "VectorVisualization with name '%s' did not find a 'position_layer' parameter.",
      name_.c_str());
    return false;
  }

  scale_ = 1.0;
  if (!nodePtr_->get_parameter(name_ + ".params.scale", scale_)) {
    RCLCPP_INFO(
      nodePtr_->get_logger(),
      "VectorVisualization with name '%s' did not find a 'scale' parameter. Using default.",
      name_.c_str());
  }

  lineWidth_ = 0.003;
  if (!nodePtr_->get_parameter(name_ + ".params.line_width", lineWidth_)) {
    RCLCPP_INFO(
      nodePtr_->get_logger(),
      "VectorVisualization with name '%s' did not find a 'line_width' parameter. Using default.",
      name_.c_str());
  }

  int colorValue = 65280;  // green
  if (!nodePtr_->get_parameter(name_ + ".params.color", colorValue)) {
    RCLCPP_INFO(
      nodePtr_->get_logger(),
      "VectorVisualization with name '%s' did not find a 'color' parameter. Using default.",
      name_.c_str());
  }
  setColorFromColorValue(color_, colorValue, true);

  return true;
}

bool VectorVisualization::initialize()
{
  marker_.ns = "vector";
  marker_.lifetime = rclcpp::Duration(0ns);  // setting lifetime forever
  marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_.scale.x = lineWidth_;
  publisher_ = nodePtr_->create_publisher<visualization_msgs::msg::Marker>(
    name_,
    rclcpp::QoS(1).transient_local());
  return true;
}

bool VectorVisualization::visualize(const grid_map::GridMap & map)
{
  if (!isActive()) {return false;}

  for (const auto & type : types_) {
    if (!map.exists(type)) {
      RCLCPP_WARN_STREAM(
        nodePtr_->get_logger(),
        "VectorVisualization::visualize: No grid map layer with name '" << type << "' found.");
      return false;
    }
  }

  // Set marker info.
  marker_.header.frame_id = map.getFrameId();
  marker_.header.stamp = rclcpp::Time(map.getTimestamp());

  // Clear points.
  marker_.points.clear();
  marker_.colors.clear();

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (!map.isValid(*iterator, positionLayer_) || !map.isValid(*iterator, types_)) {continue;}
    geometry_msgs::msg::Vector3 vector;
    vector.x = map.at(types_[0], *iterator);
    vector.y = map.at(types_[1], *iterator);
    vector.z = map.at(types_[2], *iterator);

    Eigen::Vector3d position;
    map.getPosition3(positionLayer_, *iterator, position);
    geometry_msgs::msg::Point startPoint;
    startPoint.x = position.x();
    startPoint.y = position.y();
    startPoint.z = position.z();
    marker_.points.push_back(startPoint);

    geometry_msgs::msg::Point endPoint;
    endPoint.x = startPoint.x + scale_ * vector.x;
    endPoint.y = startPoint.y + scale_ * vector.y;
    endPoint.z = startPoint.z + scale_ * vector.z;
    marker_.points.push_back(endPoint);

    marker_.colors.push_back(color_);  // Each vertex needs a color.
    marker_.colors.push_back(color_);
  }

  publisher_->publish(marker_);
  return true;
}
}  // namespace grid_map_visualization
