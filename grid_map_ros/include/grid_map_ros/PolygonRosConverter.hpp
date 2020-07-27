/*
 * PolygonRosConverter.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_ROS__POLYGONROSCONVERTER_HPP_
#define GRID_MAP_ROS__POLYGONROSCONVERTER_HPP_

#include <grid_map_core/Polygon.hpp>

// ROS
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// STL
#include <string>

namespace grid_map
{

class PolygonRosConverter
{
public:
  /*!
   * Default constructor.
   */
  PolygonRosConverter();

  /*!
   * Constructor with vertices.
   * @param vertices the points of the polygon.
   */
  virtual ~PolygonRosConverter();

  /*!
   * Converts a polygon object to a ROS PolygonStamped message.
   * @param[in] polygon the polygon object.
   * @param[out] message the ROS PolygonStamped message to be populated.
   */
  static void toMessage(
    const grid_map::Polygon & polygon,
    geometry_msgs::msg::PolygonStamped & message);

  /*!
   * Converts a polygon object to a ROS line strip marker message.
   * @param[in] polygon the polygon object.
   * @param[in] color the desired color of the marker.
   * @param[in] lineWidth the with of the line marker.
   * @param[in] zCoordinate z-coordinate of the planar polygon.
   * @param[out] marker the ROS marker message to be populated.
   */
  static void toLineMarker(
    const grid_map::Polygon & polygon, const std_msgs::msg::ColorRGBA & color,
    const double lineWidth,
    const double zCoordinate, visualization_msgs::msg::Marker & marker);

  /*!
   * Converts a polygon object to a ROS triangle list marker message.
   * @param[in] polygon the polygon object.
   * @param[in] color the desired color of the marker.
   * @param[in] zCoordinate z-coordinate of the planar polygon.
   * @param[out] marker the ROS marker message to be populated.
   */
  static void toTriangleListMarker(
    const grid_map::Polygon & polygon, const std_msgs::msg::ColorRGBA & color,
    const double zCoordinate, visualization_msgs::msg::Marker & marker);
};

}  // namespace grid_map
#endif  // GRID_MAP_ROS__POLYGONROSCONVERTER_HPP_
