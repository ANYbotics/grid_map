/*
 * Polygon.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_lib/Polygon.hpp>

// STL
#include <string>

// ROS
#include <ros/time.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

namespace grid_map {

class Polygon : public grid_map_lib::Polygon
{
 public:

  /*!
   * Default constructor.
   */
  Polygon();

  /*!
   * Constructor with vertices.
   * @param vertices the points of the polygon.
   */
  virtual ~Polygon();

  /*!
   * Set the timestamp.
   * @param timestamp the timestamp to set.
   */
  void setTimestamp(const ros::Time& timestamp);

  /*!
   * Get the timestamp.
   * @return timestamp.
   */
  const ros::Time& getTimestamp() const;

  /*!
   * Set the frame id.
   * @param frameId the frame id to set.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frameId.
   * @return frameId.
   */
  const std::string& getFrameId() const;

  /*!
   * Transforms the polygon to a ROS PolygonStamped message type.
   * @param[out] polygonMsg the ROS PolygonStamped message.
   */
  void toMessage(geometry_msgs::PolygonStamped& polygonMsg) const;

  /*!
   * Transforms the polygon to a ROS PolygonStamped message type.
   * @param[out] polygonMsg the ROS PolygonStamped message.
   */
  void toMarker(visualization_msgs::Marker& marker) const;

  /*!
   * Computes the convex hull of two polygons and returns it as polygon.
   * @param[in] polygon1 the first input polygon.
   * @param[in] polygon2 the second input polygon.
   */
  Polygon convexHull(Polygon& polygon1, Polygon& polygon2);

 private:
  /*!
   * Returns true if the vector1 and vector2 are sorted lexicographically.
   * @param[in] vector1 the first input vector.
   * @param[in] vector2 the second input vector.
   */
  static bool sortVertices(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2);

  /*!
   * Returns the 2D cross product of vector1 and vector2.
   * @param[in] vector1 the first input vector.
   * @param[in] vector2 the second input vector.
   */
  double computeCrossProduct2D(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2);

  //! Frame id of the polygon.
  std::string frameId_;

  //! Timestamp.
  ros::Time timestamp_;
};

} /* namespace grid_map */
