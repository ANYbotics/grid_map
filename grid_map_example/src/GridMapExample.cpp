/*
 * GridMapExample.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_example/GridMapExample.hpp"
#include <grid_map/GridMap.hpp>
#include <grid_map_msg/GridMap.h>
#include <grid_map_lib/iterators/GridMapIterator.hpp>

// Eigen
#include <Eigen/Core>

using namespace std;
using namespace ros;

namespace grid_map_example {

GridMapExample::GridMapExample(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("Grid map example node started.");
  publisher_ = nodeHandle_.advertise<grid_map_msg::GridMap>("iterator_demo", 1);

  demoGridMapIterator();
}

GridMapExample::~GridMapExample() {}

void GridMapExample::demoGridMapIterator()
{
  ROS_INFO("Running grid map iterator demo.");

  // Setting up map.
  std::vector<std::string> types;
  types.push_back("type");
  grid_map::GridMap map(types);
  map.setGeometry(Eigen::Array2d(1.0, 1.0), 0.05, Eigen::Vector2d(0.0, 0.0));

  // Iterating.
  for (grid_map_lib::GridMapIterator iterator(map); !iterator.isPassedEnd(); ++iterator) {

    // Setting data.
    map.at("type", *iterator) = 1.0;
    map.setTimestamp(ros::Time::now().toNSec());

    // Publish map.
    grid_map_msg::GridMap message;
    map.toMessage(message);
    publisher_.publish(message);
    ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    ros::Duration duration(0.02);
    duration.sleep();
  }
}

} /* namespace */
