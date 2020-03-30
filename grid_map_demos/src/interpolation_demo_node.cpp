/*
 * interpolation_demo_node.cpp
 *
 *  Created on: Mar 16, 2020
 *      Author: jelavice
 */

#include "grid_map_demos/InterpolationDemo.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_interpolation_demo");
  ros::NodeHandle nodeHandle("~");
  grid_map_demos::InterpolationDemo interpolationDemo(&nodeHandle);
  ros::spin();
  return 0;
}
