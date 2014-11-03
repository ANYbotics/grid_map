/*
 * OccupancyGridVisualization.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/OccupancyGridVisualization.hpp"

#include <nav_msgs/OccupancyGrid.h>

namespace grid_map_visualization {

OccupancyGridVisualization::OccupancyGridVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  occupancyGridPublisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1, true);
}

OccupancyGridVisualization::~OccupancyGridVisualization()
{

}

bool OccupancyGridVisualization::readParameters()
{
  nodeHandle_.param("occupancy_grid/type", gridType_, std::string("occupancy"));
  nodeHandle_.param("occupancy_grid/data_min", dataMin_, 0.0);
  nodeHandle_.param("occupancy_grid/data_max", dataMax_, 1.0);
  return true;
}

bool OccupancyGridVisualization::initialize()
{
  return true;
}

bool OccupancyGridVisualization::visualize(const grid_map::GridMap& map)
{
  if (occupancyGridPublisher_.getNumSubscribers () < 1) return true;
  nav_msgs::OccupancyGrid occupancyGrid;
  map.toOccupancyGrid(occupancyGrid, gridType_, dataMin_, dataMax_);
  occupancyGridPublisher_.publish(occupancyGrid);
  return true;
}

} /* namespace */
