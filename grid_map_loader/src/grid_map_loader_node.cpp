/*
 * grid_map_loader_node.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_loader/GridMapLoader.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_loader");
  ros::NodeHandle nodeHandle("~");
  grid_map_loader::GridMapLoader gridMapLoader(nodeHandle);
  ros::waitForShutdown();
  return 0;
}
