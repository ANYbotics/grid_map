/*
 * PointCloud2_to_GridMap_msg_node.cpp
 *
 *  Created on: June 25, 2021
 *      Author: Maximilian Stölzle
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/PointCloud2_to_GridMap_msg_node.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

PointCloud2ToGridMapMsgNode::PointCloud2ToGridMapMsgNode(ros::NodeHandle& nodeHandle)
{
  ROS_INFO("PointCloud2ToGridMapMsgNode started");
  nodeHandle_ = nodeHandle;

  // Publisher
  nodeHandle_.param<std::string>("grid_map_topic", gridMapTopic_, "grid_map");
  pub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(gridMapTopic_, 1, true);

  // Subscriber
  nodeHandle_.param<std::string>("point_cloud_topic", pointCloudTopic_, "point_cloud");
  sub_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &PointCloud2ToGridMapMsgNode::sub_callback, this);

}

PointCloud2ToGridMapMsgNode::~PointCloud2ToGridMapMsgNode()
{
  ROS_INFO("PointCloud2ToGridMapMsgNode deconstructed");
}

void PointCloud2ToGridMapMsgNode::sub_callback(const sensor_msgs::PointCloud2 & point_cloud_msg) 
{
  ROS_INFO("Received PointCloud2 message");
  // init GridMapPclLoader
  grid_map::GridMapPclLoader gridMapPclLoader;
  gridMapPclLoader.loadParameters(gm::getParameterPath());

  // load PCL point cloud from message
  gridMapPclLoader.loadCloudFromMessage(point_cloud_msg);
  gm::processPointcloud(&gridMapPclLoader, nodeHandle_);

  // create grid map
  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(nodeHandle_));

  // publish grid map msg
  grid_map_msgs::GridMap grid_map_msg;
  grid_map::GridMapRosConverter::toMessage(gridMap, grid_map_msg);
  pub_.publish(grid_map_msg);
}

int main(int argc, char** argv) {
  ROS_INFO("Launched PointCloud2_to_GridMap_msg_node");

  ros::init(argc, argv, "PointCloud2_to_GridMap_msg_node");
  ros::NodeHandle nodeHandle("~");
  gm::setVerbosityLevelToDebugIfFlagSet(nodeHandle);

  PointCloud2ToGridMapMsgNode node = PointCloud2ToGridMapMsgNode(nodeHandle);

  // run
  ros::spin();
  return EXIT_SUCCESS;
}
