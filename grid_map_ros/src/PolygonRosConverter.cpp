/*
 * PolygonRosConverter.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

// ROS
#include <geometry_msgs/msg/point32.h>

#include <vector>

#include "grid_map_ros/PolygonRosConverter.hpp"

using namespace std::chrono_literals;

namespace grid_map
{

PolygonRosConverter::PolygonRosConverter() {}

PolygonRosConverter::~PolygonRosConverter() {}

void PolygonRosConverter::toMessage(
  const grid_map::Polygon & polygon,
  geometry_msgs::msg::PolygonStamped & message)
{
  rclcpp::Time time_stamp(polygon.getTimestamp());
  message.header.stamp = time_stamp;
  message.header.frame_id = polygon.getFrameId();

  for (const auto & vertex : polygon.getVertices()) {
    geometry_msgs::msg::Point32 point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = 0.0;
    message.polygon.points.push_back(point);
  }
}

void PolygonRosConverter::toLineMarker(
  const grid_map::Polygon & polygon, const std_msgs::msg::ColorRGBA & color, const double lineWidth,
  const double zCoordinate, visualization_msgs::msg::Marker & marker)
{
  rclcpp::Time time_stamp(polygon.getTimestamp());
  marker.header.stamp = time_stamp;
  marker.header.frame_id = polygon.getFrameId();
  marker.lifetime = rclcpp::Duration(0.0ns);
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.color = color;
  marker.scale.x = lineWidth;

  unsigned int startIndex = marker.points.size();
  unsigned int nVertices = polygon.nVertices() + 1;
  unsigned int nTotalVertices = marker.points.size() + nVertices;
  marker.points.resize(nTotalVertices);
  marker.colors.resize(nTotalVertices, color);

  unsigned int i = startIndex;
  for ( ; i < nTotalVertices - 1; i++) {
    marker.points[i].x = polygon[i].x();
    marker.points[i].y = polygon[i].y();
    marker.points[i].z = zCoordinate;
  }
  marker.points[i].x = marker.points[startIndex].x;
  marker.points[i].y = marker.points[startIndex].y;
  marker.points[i].z = marker.points[startIndex].z;
}

void PolygonRosConverter::toTriangleListMarker(
  const grid_map::Polygon & polygon, const std_msgs::msg::ColorRGBA & color,
  const double zCoordinate, visualization_msgs::msg::Marker & marker)
{
  rclcpp::Time time_stamp(polygon.getTimestamp());
  marker.header.stamp = time_stamp;
  marker.header.frame_id = polygon.getFrameId();
  marker.lifetime = rclcpp::Duration(0.0ns);
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color = color;

  std::vector<Polygon> polygons = polygon.triangulate();
  if (polygons.size() < 1) {return;}

  size_t nPoints = 3 * polygons.size();
  marker.points.resize(nPoints);
  marker.colors.resize(polygons.size(), color);

  for (size_t i = 0; i < polygons.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) {
      const size_t pointIndex = 3 * i + j;
      marker.points[pointIndex].x = polygons[i].getVertex(j).x();
      marker.points[pointIndex].y = polygons[i].getVertex(j).y();
      marker.points[pointIndex].z = zCoordinate;
    }
  }
}

}  // namespace grid_map
