/*
 * PolygonRosConverter.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_core/Polygon.hpp>

// STL
#include <string>

// ROS
#include <ros/time.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

namespace grid_map {

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
  static void toMessage(const grid_map::Polygon& polygon, geometry_msgs::PolygonStamped& message);

  /*!
   * Converts a polygon object to a ROS Marker message.
   * @param[in] polygon the polygon object.
   * @param[out] marker the ROS Marker message to be populated.
   */
  static void toMarker(const grid_map::Polygon& polygon, const std_msgs::ColorRGBA& color,
                       const double lineWidth, visualization_msgs::Marker& marker);
};

} /* namespace grid_map */
