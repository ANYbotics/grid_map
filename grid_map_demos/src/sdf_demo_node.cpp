/*
* sdf_demo_node.cpp
*
*  Created on: May 3, 2022
*      Author: Ruben Grandia
*   Institute: ETH Zurich
*/

#include <string>

#include <ros/ros.h>

#include "grid_map_demos/SdfDemo.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_sdf_demo");
  ros::NodeHandle nodeHandle("");

  std::string elevationLayer;
  nodeHandle.getParam("elevation_layer", elevationLayer);

  std::string mapTopic;
  nodeHandle.getParam("grid_map_topic", mapTopic);

  std::string pointcloudTopic;
  nodeHandle.getParam("pointcloud_topic", pointcloudTopic);

  grid_map_demos::SdfDemo sdfDemo(nodeHandle, mapTopic, elevationLayer, pointcloudTopic);

  ros::spin();
  return 0;
}