/*
 * pointcloud_publisher_node.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;
using Point = ::pcl::PointXYZ;
using PointCloud = ::pcl::PointCloud<Point>;

void publishCloud(const std::string& filename, const ros::Publisher& pub, const std::string& frame) {
  PointCloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud = gm::loadPointcloudFromPcd(filename);
  cloud->header.frame_id = frame;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  ROS_INFO_STREAM("Publishing loaded cloud, number of points: " << cloud->points.size());
  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_pub_node");
  ros::NodeHandle nh("~");

  const std::string pathToCloud = gm::getPcdFilePath(nh);
  const std::string cloudFrame = nh.param<std::string>("cloud_frame", "");
  // publish cloud
  ros::Publisher cloudPub = nh.advertise<sensor_msgs::PointCloud2>("raw_pointcloud", 1, true);
  publishCloud(pathToCloud, cloudPub, cloudFrame);

  // run
  ros::spin();
  return EXIT_SUCCESS;
}
