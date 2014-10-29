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
#include <nav_msgs/OccupancyGrid.h>

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
   * Gets a submap from the map. The requested submap is specified with the requested
   * location and length.
   * @param[in] position the requested position of the submap (usually the center).
   * @param[in] length the requested length of the submap.
   * @param[out] indexInSubmap the index of the requested position in the submap.
   * @param[out] isSuccess true if successful, false otherwise.
   * @return submap (is empty if success is false).
   */
  GridMap getSubmap(const Eigen::Vector2d& position, const Eigen::Array2d& length, Eigen::Array2i& indexInSubmap, bool& isSuccess);

  /*!
   * Puts the all contents to a ROS message of type GridMap.
   * @param[out] message the GridMap message to be populated.
   */
  void toMessage(grid_map_msg::GridMap& message) const;

  /*!
   * Puts the data of requested types to a ROS message of type GridMap.
   * @param[in] types the types to be added to the message.
   * @param[out] message the GridMap message to be populated.
   */
  void toMessage(const std::vector<std::string>& types, grid_map_msg::GridMap& message) const;

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

  /*!
   * Puts the contents to a ROS OccupancyGrid message. Set the type to be transformed
   * as the cell data of the occupancy grid, all other types will be neglected.
   * @param occupancyGrid the message to be populated.
   * @param cellType the type that is transformed to the occupancy cell data.
   * @param dataMin the minimum value of the grid map data (used to normalize the cell data in [min, max]).
   * @param dataMax the maximum value of the grid map data (used to normalize the cell data in [min, max]).
   */
  void toOccupancyGrid(nav_msgs::OccupancyGrid& occupancyGrid, const std::string& cellType,
                       float dataMin, float dataMax) const;

 private:
  /*!
   * Sets the contents from a ROS message of type GridMap.
   * @param message the GridMap message.
   * @return true if successful, false otherwise.
   */
  bool fromMessage(const grid_map_msg::GridMap& message);
};

} /* namespace */
