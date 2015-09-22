/*
 * gridmap_to_image_demo_node.cpp
 *
 *  Created on: Sept 18, 2015
 *      Author: Elena Stumm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "grid_map_demos/GridmapRegistrationDemo.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "gridmap_to_image_demo");
  ros::NodeHandle nodeHandle("~");
  grid_map_demos::GridmapRegistrationDemo GridmapRegistrationDemo(nodeHandle);

  ros::spin();
  return 0;
}
