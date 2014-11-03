/*
 * VectorVisualization.cpp
 *
 *  Created on: Sep 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/VectorVisualization.hpp"

// Color conversion
#include <grid_map_visualization/GridMapVisualizationHelpers.hpp>

// Iterator
#include <grid_map_lib/iterators/GridMapIterator.hpp>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace grid_map_visualization {

VectorVisualization::VectorVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("vector", 1, true);
}

VectorVisualization::~VectorVisualization()
{

}

bool VectorVisualization::readParameters()
{
  std::string typePrefix;
  nodeHandle_.param("vector/type_prefix", typePrefix, std::string(""));
  if (typePrefix.empty()) return true;

  types_.push_back(typePrefix + "_x");
  types_.push_back(typePrefix + "_y");
  types_.push_back(typePrefix + "_z");

  nodeHandle_.param("vector/position_type", positionType_, std::string(""));

  nodeHandle_.param("vector/scale", scale_, 0.02);
  nodeHandle_.param("vector/line_width", lineWidth_, 0.001);

  int colorValue;
  nodeHandle_.param("vector/color", colorValue, 65280); // green
  setColorFromColorValue(color_, colorValue, true);

  return true;
}

bool VectorVisualization::initialize()
{
  marker_.ns = "vector";
  marker_.lifetime = ros::Duration();
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::LINE_LIST;
  marker_.scale.x = lineWidth_;
  return true;
}

bool VectorVisualization::visualize(const grid_map::GridMap& map)
{
  if (markerPublisher_.getNumSubscribers () < 1) return true;

  // Set marker info.
  marker_.header.frame_id = map.getFrameId();
  marker_.header.stamp.fromNSec(map.getTimestamp());

  // Clear points.
  marker_.points.clear();
  marker_.colors.clear();

  for (grid_map_lib::GridMapIterator iterator(map); !iterator.isPassedEnd(); ++iterator)
  {
    if (!map.isValid(*iterator) || !map.isValid(*iterator, types_)) continue;

    geometry_msgs::Vector3 vector;
    vector.x = map.at(types_[0], *iterator);
    vector.y = map.at(types_[1], *iterator);
    vector.z = map.at(types_[2], *iterator);

    Eigen::Vector3d position;
    map.getPosition3d(positionType_, *iterator, position);
    geometry_msgs::Point startPoint;
    startPoint.x = position.x();
    startPoint.y = position.y();
    startPoint.z = position.z();
    marker_.points.push_back(startPoint);

    geometry_msgs::Point endPoint;
    endPoint.x = startPoint.x + scale_ * vector.x;
    endPoint.y = startPoint.y + scale_ * vector.y;
    endPoint.z = startPoint.z + scale_ * vector.z;
    marker_.points.push_back(endPoint);

    marker_.colors.push_back(color_); // Each vertex needs a color.
    marker_.colors.push_back(color_);
  }

  markerPublisher_.publish(marker_);
  return true;
}

} /* namespace */
