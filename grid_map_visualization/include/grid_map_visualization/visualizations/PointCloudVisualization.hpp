/*
 * PointCloudVisualization.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map/GridMap.hpp>

// ROS
#include <ros/ros.h>

namespace grid_map_visualization {

class PointCloudVisualization
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PointCloudVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~PointCloudVisualization();

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

  //! ROS publisher of the point cloud.
  ros::Publisher pointCloudPublisher_;

  //! Type that is transformed to points.
  std::string pointType_;
};

} /* namespace */
