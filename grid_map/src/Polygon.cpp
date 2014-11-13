/*
 * Polygon.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <grid_map/Polygon.hpp>

// ROS
#include <geometry_msgs/Point32.h>

namespace grid_map {

Polygon::Polygon() {}

Polygon::~Polygon() {}

void Polygon::setTimestamp(const ros::Time& timestamp)
{
  timestamp_ = timestamp;
}

const ros::Time& Polygon::getTimestamp() const
{
  return timestamp_;
}

void Polygon::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

const std::string& Polygon::getFrameId() const
{
  return frameId_;
}

void Polygon::toMessage(geometry_msgs::PolygonStamped& polygonMsg) const
{
  polygonMsg.header.stamp = timestamp_;
  polygonMsg.header.frame_id = frameId_;

  for (const auto& vertex : vertices_) {
    geometry_msgs::Point32 point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = 0.0;
    polygonMsg.polygon.points.push_back(point);
  }
}

void Polygon::toMarker(visualization_msgs::Marker& marker) const
{
//    marker.header.stamp = timestamp_;
//    marker.header.frame_id = frameId_;
//    marker.lifetime = ros::Duration(0.0);
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.type = visualization_msgs::Marker::LINE_STRIP;
//    marker.color = color;
//    marker.scale.x = visualizationLineWidth_;
//
//    unsigned int startIndex = marker.points.size();
//    unsigned int nVertices = polygon.size() + 1;
//    unsigned int nTotalVertices = marker.points.size() + nVertices;
//    marker.points.resize(nTotalVertices);
//  //  marker.colors.resize(nTotalVertices, color);
//
//    unsigned int i = startIndex;
//    for( ; i < nTotalVertices - 1; i++) {
//      marker.points[i].x = polygon[i].x();
//      marker.points[i].y = polygon[i].y();
//      marker.points[i].z = plane.getHeight(polygon[i]) + visualizationOffset_;
//    }
//    marker.points[i].x = marker.points[startIndex].x;
//    marker.points[i].y = marker.points[startIndex].y;
//    marker.points[i].z = marker.points[startIndex].z;
}

} /* namespace grid_map */
