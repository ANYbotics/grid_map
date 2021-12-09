/*
 * grid_map_iterators_demo_node.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <ros/ros.h>
#include "grid_map_demos/IteratorsDemo.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_iterators_demo");

  ros::NodeHandle nodeHandle("~");
  grid_map_demos::IteratorsDemo iteratorsDemo(nodeHandle);

  ros::requestShutdown();
  return 0;
}
