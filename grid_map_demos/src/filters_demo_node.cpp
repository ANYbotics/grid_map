/*
 * filters_demo_node.cpp
 *
 *  Created on: Aug 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_demos/FiltersDemo.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "grid_map_filters_demo");
  ros::NodeHandle nodeHandle("~");
  bool success;
  grid_map_demos::FiltersDemo filtersDemo(nodeHandle, success);
  if (success) {ros::spin();}
  return 0;
}
