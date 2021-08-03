/*
 * GridMapToImageDemo.cpp
 *
 *  Created on: October 19, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_demos/GridmapToImageDemo.hpp"

#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>

namespace grid_map_demos {

GridMapToImageDemo::GridMapToImageDemo(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  readParameters();
  gridMapSubscriber_ = nodeHandle_.subscribe(gridMapTopic_, 1, &GridMapToImageDemo::gridMapCallback, this);
  ROS_ERROR("Subscribed to %s", nodeHandle_.resolveName(gridMapTopic_).c_str());
}

GridMapToImageDemo::~GridMapToImageDemo()=default;

void GridMapToImageDemo::readParameters()
{
  nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/grid_map"));
  nodeHandle_.param("file", filePath_, std::string("~/.ros/grid_map.png"));
}

void GridMapToImageDemo::gridMapCallback(const grid_map_msgs::GridMap& msg)
{
  ROS_INFO("Saving map received from: %s to file %s.", nodeHandle_.resolveName(gridMapTopic_).c_str(), filePath_.c_str());
  grid_map::GridMap map;
  cv_bridge::CvImage image;
  grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
  grid_map::GridMapRosConverter::toCvImage(map,"elevation", sensor_msgs::image_encodings::MONO8, image);
  bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
  ROS_INFO("Success writing image: %s", success?"true":"false");
  ros::shutdown();
}

} /* namespace */
