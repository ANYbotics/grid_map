/*
 * GridmapRegistrationDemo.hpp
 *
 *  Created on: Sept 18, 2015
 *      Author: Elena Stumm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <grid_map/grid_map.hpp>

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace grid_map_demos {

/*!
 * Converts the 'elevation' layer of a local and global grid map to images.
 * The local grid map is then registered w.r.t the global map, and displayed.
 * The global grid map is published and can be viewed in Rviz.
 */
class GridmapRegistrationDemo
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GridmapRegistrationDemo(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~GridmapRegistrationDemo();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  /*!
  * Reads and verifies the global map and its parameters.
  * @return true if successful.
  */
  bool loadGlobalMap();

  void gridMapCallback(const grid_map_msgs::GridMap& localMap);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Grid map data.
  grid_map::GridMap globalMap_;
  grid_map::GridMap localMap_;

  //! Image subscriber
  ros::Subscriber gridMapSubscriber_;

  //! Name of the bag and publisher topics.
  std::string localBagTopic_;
  std::string globalBagTopic_;
  std::string publishTopic_;

  //! Name of the bagfile to load the global map from.
  std::string bagFilePath_;


};

} /* namespace */
