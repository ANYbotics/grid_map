/*
 * image_to_gridmap_demo_node.cpp
 *
 *  Created on: May 04, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#include <ros/ros.h>
#include "grid_map_demos/ImageToGridmapDemo.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "image_to_gridmap_demo");
  ros::NodeHandle nh("~");
  grid_map_demos::ImageToGridmapDemo imageToGridmapDemo(nh);

  ros::spin();
  return 0;
}
