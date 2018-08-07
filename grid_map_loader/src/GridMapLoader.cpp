/*
 * GridMapLoader.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#include <grid_map_loader/GridMapLoader.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace grid_map;

namespace grid_map_loader {

GridMapLoader::GridMapLoader(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{
  readParameters();
  publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(publishTopic_, 1, true);
  if (!load()) return;
  publish();
  ros::requestShutdown();
}

GridMapLoader::~GridMapLoader()
{
}

bool GridMapLoader::readParameters()
{
  nodeHandle_.param("bag_topic", bagTopic_, std::string("/grid_map"));
  nodeHandle_.param("publish_topic", publishTopic_, bagTopic_);
  nodeHandle_.param("file_path", filePath_, std::string());
  double durationInSec;
  nodeHandle_.param("duration", durationInSec, 5.0);
  duration_.fromSec(durationInSec);
  return true;
}

bool GridMapLoader::load()
{
  ROS_INFO_STREAM("Loading grid map from path " << filePath_ << ".");
  return GridMapRosConverter::loadFromBag(filePath_, bagTopic_, map_);
}

void GridMapLoader::publish()
{
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  publisher_.publish(message);
  duration_.sleep();
}

} /* namespace */
