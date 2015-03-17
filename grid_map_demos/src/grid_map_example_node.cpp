/*
 * grid_map_example_node.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "grid_map_example/GridMapExample.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_example");

  ros::NodeHandle nodeHandle("~");
  grid_map_example::GridMapExample gridMapExample(nodeHandle);

  ros::requestShutdown();
  return 0;
}
