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
#include <grid_map_msg/GridMapCircularBuffer.h>

// STL
#include <vector>
#include <unordered_map>

// Eigen
#include <Eigen/Core>

// ROS
#include <ros/ros.h>

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
   * Destructor.
   */
  virtual ~GridMap();

  /*!
   * Sets the contents from a ROS message of type GridMap.
   * @param message the GridMap message.
   * @return true if successful, false otherwise.
   */
  bool fromMessage(const grid_map_msg::GridMap& message);

  /*!
   * Sets the contents from a ROS message of type GridMapCircularBuffer.
   * @param message the GridMapCircularBuffer message.
   * @return true if successful, false otherwise.
   */
  bool fromMessage(const grid_map_msg::GridMapCircularBuffer& message);

  /*!
   * Copies the contents to a ROS message of type GridMap.
   * @param message the GridMap message to be populated.
   */
  void toMessage(grid_map_msg::GridMap& message);

  /*!
   * Copies the contents to a ROS message of type GridMapCircularBuffer.
   * @param message the GridMapCircularBuffer message to be populated.
   */
  void toMessage(grid_map_msg::GridMapCircularBuffer& message);
};

} /* namespace */
