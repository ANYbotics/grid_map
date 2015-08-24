/*
 * GridMapLoader.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
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
  publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(topic_, 1, true);
  if (!load()) return;
  publish();
}

GridMapLoader::~GridMapLoader()
{
}

bool GridMapLoader::readParameters()
{
  nodeHandle_.param("topic", topic_, std::string("/grid_map"));
  nodeHandle_.param("file_path", filePath_, std::string(""));
  return true;
}

bool GridMapLoader::load()
{
  ROS_INFO_STREAM("Loading grid map from path " << filePath_ << ".");
  map_.add("elevation", 0.69);
//  return GridMapRosConverter::loadFromBag(filePath_, topic_, map_);
}

void GridMapLoader::publish()
{
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  publisher_.publish(message);
}

} /* namespace */
