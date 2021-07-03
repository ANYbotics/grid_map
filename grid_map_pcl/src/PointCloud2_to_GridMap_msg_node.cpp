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
  ROS_INFO("PointCloud2ToGridMapMsgNode started.");
  nodeHandle_ = nodeHandle;

  // activityCheckTimer_ = nodeHandle_.createTimer(activityCheckDuration_,
  //                                               &GridMapVisualization::updateSubscriptionCallback,
  //                                               this);

  // Publisher
  nodeHandle_.param<std::string>("grid_map_topic", gridMapTopic_, "grid_map");
  pub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(gridMapTopic_, 1, true);

  // // Subscriber
  nodeHandle_.param<std::string>("point_cloud_topic", pointCloudTopic_, "point_cloud");
  sub_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &PointCloud2ToGridMapMsgNode::sub_callback, this);

}

PointCloud2ToGridMapMsgNode::~PointCloud2ToGridMapMsgNode()
{
  ROS_INFO("PointCloud2ToGridMapMsgNode deconstructed.");
}

void PointCloud2ToGridMapMsgNode::sub_callback(const sensor_msgs::PointCloud2 & msg) 
{

}

// void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//   // gridMapPclLoader.loadCloudFromMessage(pathToCloud);

//   // grid_map::GridMapPclLoader gridMapPclLoader;
//   // gridMapPclLoader.loadParameters(gm::getParameterPath());
//   // gridMapPclLoader.loadCloudFromMessage(pathToCloud);

//   // gm::processPointcloud(&gridMapPclLoader, nh);

//   // grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
//   // gridMap.setFrameId(gm::getMapFrame(nh));

//   // // publish grid map
//   // grid_map_msgs::GridMap msg;
//   // grid_map::GridMapRosConverter::toMessage(gridMap, msg);
//   // pub.publish(msg);
// }

int main(int argc, char** argv) {
  ROS_INFO("Launched PointCloud2_to_GridMap_msg_node");

  ros::init(argc, argv, "PointCloud2_to_GridMap_msg_node");
  ros::NodeHandle nodeHandle("~");
  gm::setVerbosityLevelToDebugIfFlagSet(nodeHandle);

  // // Publisher
  // std::string gridMapTopic;
  // nh.param<std::string>("grid_map_topic", gridMapTopic, "grid_map");
  // ros::Publisher pub;
  // pub = nh.advertise<grid_map_msgs::GridMap>(gridMapTopic, 1, true);

  // // Subscriber
  // std::string pointCloudTopic;
  // nh.param<std::string>("point_cloud_topic", pointCloudTopic, "point_cloud");
  // ros::Subscriber sub = nh.subscribe(pointCloudTopic, 1, callback);

  PointCloud2ToGridMapMsgNode node = PointCloud2ToGridMapMsgNode(nodeHandle);

  // run
  ros::spin();
  return EXIT_SUCCESS;
}
