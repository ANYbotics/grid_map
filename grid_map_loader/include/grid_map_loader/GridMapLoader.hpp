/*
 * GridMapLoader.hpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#pragma once

// ROS
#include <ros/ros.h>

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>

// STD
#include <string>

namespace grid_map_loader {

/*!
 * Loads and publishes a grid map from a bag file.
 */
class GridMapLoader
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GridMapLoader(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~GridMapLoader();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  /*!
   * Loads the grid map from the bag file.
   * @return true if successful, false otherwise.
   */
  bool load();

  /*!
   * Publishes the grid map.
   */
  void publish();

 private:

  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

  //! Grid map publisher.
  ros::Publisher publisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Path the ROS bag to be published.
  std::string filePath_;

  //! Topic name of the grid map in the ROS bag.
  std::string bagTopic_;

  //! Topic name of the grid map to be loaded.
  std::string publishTopic_;

  //! Duration to publish the grid map.
  ros::Duration duration_;
};

} /* namespace */
