/*
 * PolygonRosConverter.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "grid_map_ros/PolygonRosConverter.hpp"

// ROS
#include <geometry_msgs/Point32.h>

namespace grid_map {

PolygonRosConverter::PolygonRosConverter() {}

PolygonRosConverter::~PolygonRosConverter() {}

void PolygonRosConverter::toMessage(const grid_map::Polygon& polygon, geometry_msgs::PolygonStamped& message)
{
  message.header.stamp.fromNSec(polygon.getTimestamp());
  message.header.frame_id = polygon.getFrameId();

  for (const auto& vertex : polygon.getVertices()) {
    geometry_msgs::Point32 point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = 0.0;
    message.polygon.points.push_back(point);
  }
}

void PolygonRosConverter::toMarker(const grid_map::Polygon& polygon,
                                   const std_msgs::ColorRGBA& color, const double lineWidth,
                                   visualization_msgs::Marker& marker)
{
    marker.header.stamp.fromNSec(polygon.getTimestamp());
    marker.header.frame_id = polygon.getFrameId();
    marker.lifetime = ros::Duration(0.0);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.color = color;
    marker.scale.x = lineWidth;

    unsigned int startIndex = marker.points.size();
    unsigned int nVertices = polygon.nVertices() + 1;
    unsigned int nTotalVertices = marker.points.size() + nVertices;
    marker.points.resize(nTotalVertices);
    marker.colors.resize(nTotalVertices, color);

    unsigned int i = startIndex;
    for( ; i < nTotalVertices - 1; i++) {
      marker.points[i].x = polygon[i].x();
      marker.points[i].y = polygon[i].y();
      marker.points[i].z = 0.0; // TODO: Consider adding option for offset.
    }
    marker.points[i].x = marker.points[startIndex].x;
    marker.points[i].y = marker.points[startIndex].y;
    marker.points[i].z = marker.points[startIndex].z;
}

} /* namespace grid_map */
