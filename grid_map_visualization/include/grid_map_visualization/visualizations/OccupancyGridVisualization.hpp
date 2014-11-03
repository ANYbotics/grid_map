/*
 * PointCloudOccupancyGrid.hpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map/GridMap.hpp>

// ROS
#include <ros/ros.h>

namespace grid_map_visualization {

class OccupancyGridVisualization
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  OccupancyGridVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~OccupancyGridVisualization();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   */
  bool initialize();

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap& map);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS publisher of the occupancy grid.
  ros::Publisher occupancyGridPublisher_;

  //! Type that is transformed to the occupancy grid.
  std::string gridType_;

  //! Minimum and maximum value of the grid map data (used to normalize the cell data in [min, max]).
  double dataMin_, dataMax_;
};

} /* namespace */
