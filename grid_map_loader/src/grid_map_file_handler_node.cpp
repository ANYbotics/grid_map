/*
 * grid_map_loader_node.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>

#include "../include/grid_map_file_handler/GridMapFileHandler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_loader");
  ros::NodeHandle nodeHandle("~");
  grid_map_loader::GridMapLoader gridMapLoader(nodeHandle);
  ros::waitForShutdown();
  return 0;
}
