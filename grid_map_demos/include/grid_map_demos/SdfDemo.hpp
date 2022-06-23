/*
 * SdfDemo.hpp
 *
 *  Created on: May 3, 2022
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#pragma once

#include <string>

// ROS
#include <ros/ros.h>

// gridmap
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>

namespace grid_map_demos {

/**
 * Subscribes to a gridmap, converts it to a signed distance field, and publishes the signed distance field as a pointcloud.
 */
class SdfDemo {
 public:

  /**
   * Constructor.
   * @param nodeHandle : the ROS node handle.
   * @param mapTopic : grid map topic to subscribe to
   * @param elevationLayer : name of the elevation layer
   * @param pointcloudTopic : point cloud topic to publish the result to
   */
  SdfDemo(ros::NodeHandle& nodeHandle, const std::string& mapTopic, std::string elevationLayer, const std::string& pointcloudTopic);

 private:
  void callback(const grid_map_msgs::GridMap& message);

  //! Grid map subscriber.
  ros::Subscriber gridmapSubscriber_;

  //! Pointcloud publisher.
  ros::Publisher pointcloudPublisher_;

  //! Free space publisher.
  ros::Publisher freespacePublisher_;

  //! Occupied space publisher.
  ros::Publisher occupiedPublisher_;

  //! Elevation layer in the received grid map
  std::string elevationLayer_;
};

}  // namespace grid_map_demos