/*
 * GridMapExample.hpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// ROS
#include <ros/ros.h>

namespace grid_map_example {

/*!
 * Visualizes a grid map by publishing different topics that can be viewed in Rviz.
 */
class GridMapExample
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GridMapExample(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~GridMapExample();

  /*!
   * Callback function for the grid map.
   * @param message the grid map message to be visualized.
   */
  void demoGridMapIterator();

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher publisher_;
};

} /* namespace */
