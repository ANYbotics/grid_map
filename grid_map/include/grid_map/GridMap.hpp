/*
 * GridMap.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_lib/GridMap.hpp>
#include <grid_map_msg/GridMap.h>

// STL
#include <vector>
#include <unordered_map>

// Eigen
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace grid_map {

/*!
 * ROS interface for the Grid Map library.
 */
class GridMap : public grid_map_lib::GridMap
{
 public:
  /*!
   * Constructor.
   * @param types a vector of strings containing the definition/description of the data.
   */
  GridMap(const std::vector<std::string>& types);

  /*!
   * Constructor. Sets the contents from a ROS message of type GridMap.
   * @param message the GridMap message.
   */
  GridMap(const grid_map_msg::GridMap& message);

  /*!
   * Destructor.
   */
  virtual ~GridMap();

  /*!
   * Puts the contents to a ROS message of type GridMap.
   * @param message the GridMap message to be populated.
   */
  void toMessage(grid_map_msg::GridMap& message) const;

  /*!
   * Puts the contents to a ROS PointCloud2 message. Set the type to be transformed
   * as the points of the point cloud, all other types will be added as additional fields.
   * @param pointCloud the message to be populated.
   * @param pointType the type that is transformed to points.
   */
  void toPointCloud(sensor_msgs::PointCloud2& pointCloud, const std::string& pointType) const;

  /*!
   * Puts the contents to a ROS PointCloud2 message. Set the type to be transformed
   * as the points of the point cloud and all other types to be added as additional fields.
   * @param pointCloud the message to be populated.
   * @param pointType the type that is transformed to points.
   * @param typesToAdd the types that should be added as fields to the point cloud. Must include the pointType.
   */
  void toPointCloud(sensor_msgs::PointCloud2& pointCloud, const std::string& pointType,
                    const std::vector<std::string>& typesToAdd) const;

 private:
  /*!
   * Sets the contents from a ROS message of type GridMap.
   * @param message the GridMap message.
   * @return true if successful, false otherwise.
   */
  bool fromMessage(const grid_map_msg::GridMap& message);
};

} /* namespace */
