/*
 * GridMapVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/GridMapVisualization.hpp"

#include <grid_map_lib/GridMap.hpp>

using namespace std;
using namespace ros;

namespace grid_map_visualization {

GridMapVisualization::GridMapVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      mapRegionVisualization_(nodeHandle),
      pointCloudVisualization_(nodeHandle),
      vectorVisualization_(nodeHandle),
      occupancyGridVisualization_(nodeHandle)
{
  ROS_INFO("Grid map visualization node started.");
  readParameters();
  mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &GridMapVisualization::callback, this);
  initialize();
}

GridMapVisualization::~GridMapVisualization()
{

}

bool GridMapVisualization::readParameters()
{
  nodeHandle_.param("grid_map_topic", mapTopic_, string("/grid_map"));
  mapRegionVisualization_.readParameters();
  pointCloudVisualization_.readParameters();
  vectorVisualization_.readParameters();
  occupancyGridVisualization_.readParameters();
  return true;
}

bool GridMapVisualization::initialize()
{
  mapRegionVisualization_.initialize();
  pointCloudVisualization_.initialize();
  vectorVisualization_.initialize();
  occupancyGridVisualization_.initialize();
  ROS_INFO("Grid map visualization initialized.");
  return true;
}

void GridMapVisualization::callback(const grid_map_msg::GridMap& message)
{
  ROS_DEBUG("Grid map visualization received a map (timestamp %f) for visualization.", message.info.header.stamp.toSec());
  grid_map::GridMap map(message);
  mapRegionVisualization_.visualize(map);
  pointCloudVisualization_.visualize(map);
  vectorVisualization_.visualize(map);
  occupancyGridVisualization_.visualize(map);
}

} /* namespace */
