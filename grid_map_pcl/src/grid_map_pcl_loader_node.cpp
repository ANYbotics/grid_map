/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map_pcl_loader_node");
  ros::NodeHandle nh("~");
  gm::setVerbosityLevelToDebugIfFlagSet(nh);

  ros::Publisher gridMapPub;
  gridMapPub = nh.advertise<grid_map_msgs::GridMap>("grid_map_from_raw_pointcloud", 1, true);

  grid_map::GridMapPclLoader gridMapPclLoader;
  const std::string pathToCloud = gm::getPcdFilePath(nh);
  gridMapPclLoader.loadParameters(gm::getParameterPath(nh));
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  gm::processPointcloud(&gridMapPclLoader, nh);

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(nh));

  gm::saveGridMap(gridMap, nh, gm::getMapRosbagTopic(nh));

  // publish grid map
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap, msg);
  gridMapPub.publish(msg);

  // run
  ros::spin();
  return EXIT_SUCCESS;
}
