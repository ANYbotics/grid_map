/*
 * ImageToGridmapDemo.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_demos/ImageToGridmapDemo.hpp"

#include <sensor_msgs/PointCloud2.h>

namespace grid_map_demos {

ImageToGridmapDemo::ImageToGridmapDemo(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(grid_map::GridMap({"elevation"})),
      mapInitialized_(false)
{
  readParameters();
  grayscaleMaskSub_ = nodeHandle_.subscribe(imageTopic_,1,&ImageToGridmapDemo::maskCallback, this);
  imageSub_ = nodeHandle_.subscribe(imageTopic_,1,&ImageToGridmapDemo::imageCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("image_grid_map", 1);
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("output", 1, true);
}

ImageToGridmapDemo::~ImageToGridmapDemo() {}

bool ImageToGridmapDemo::readParameters()
{
  nodeHandle_.param("elevation_map_topic", imageTopic_, std::string("/grid_map_image"));
  nodeHandle_.param("map_length_x", mapLengthX_, 1.0);
  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("map"));
  nodeHandle_.param("min_height", minHeight_, 0.0);
  nodeHandle_.param("max_height", maxHeight_, 1.0);

  return true;
}

void ImageToGridmapDemo::maskCallback(const sensor_msgs::Image& msg)
{
  map_.setFrameId(mapFrameId_);
  grid_map::GridMapRosConverter::fromImage(msg, std::string("mask"), mapLengthX_, map_);
  //std::cout << map_["mask"];
  ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).",
           map_.getLength().x(), map_.getLength().y(),
           map_.getSize()(0), map_.getSize()(1));
  mapInitialized_ = true;
  grayscaleMaskSub_.shutdown();
}

void ImageToGridmapDemo::imageCallback(const sensor_msgs::Image& msg)
{
  if (!mapInitialized_) return;
  grid_map::GridMapRosConverter::addLayerFromGrayscaleImage(msg, std::string("elevation"), map_, minHeight_, maxHeight_);
  map_.setTimestamp(msg.header.stamp.toNSec());
  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
  gridMapPublisher_.publish(mapMessage);
  // Publish as point cloud.
  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map_, "elevation", pointCloud);
  pointCloudPublisher_.publish(pointCloud);
}



} /* namespace */
