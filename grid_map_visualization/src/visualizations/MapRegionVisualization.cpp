/*
 * MapRegionVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/MapRegionVisualization.hpp"

#include <grid_map_visualization/GridMapVisualizationHelpers.hpp>

// ROS
#include "geometry_msgs/Point.h"

namespace grid_map_visualization {

MapRegionVisualization::MapRegionVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      nVertices_(5),
      lineWidth_(0.01)
{
  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("region", 1, true);
}

MapRegionVisualization::~MapRegionVisualization()
{

}

bool MapRegionVisualization::readParameters()
{
  nodeHandle_.param("map_region/line_width", lineWidth_, 0.003);

  int colorValue;
  nodeHandle_.param("map_region/color", colorValue, 16777215); // white, http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D20%2C+g%3D50%2C+b%3D230%7D
  setColorFromColorValue(color_, colorValue, true);

  return true;
}

bool MapRegionVisualization::initialize()
{
  marker_.ns = "map_region";
  marker_.lifetime = ros::Duration();
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::LINE_STRIP;
  marker_.scale.x = lineWidth_;
  marker_.points.resize(nVertices_); // Initialized to [0.0, 0.0, 0.0]
  marker_.colors.resize(nVertices_, color_);
  return true;
}

bool MapRegionVisualization::visualize(const grid_map::GridMap& map)
{
  if (markerPublisher_.getNumSubscribers () < 1) return true;

  // Set marker info.
  marker_.header.frame_id = map.getFrameId();
  marker_.header.stamp.fromNSec(map.getTimestamp());

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

  markerPublisher_.publish(marker_);

  return true;
}

} /* namespace */
