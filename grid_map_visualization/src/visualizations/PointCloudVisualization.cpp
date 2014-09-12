/*
 * PointCloudVisualization.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/PointCloudVisualization.hpp"

#include <sensor_msgs/PointCloud2.h>

namespace grid_map_visualization {

PointCloudVisualization::PointCloudVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);
}

PointCloudVisualization::~PointCloudVisualization()
{

}

bool PointCloudVisualization::readParameters()
{
  nodeHandle_.param("point_cloud/point_type", pointType_, std::string("feature"));
  return true;
}

bool PointCloudVisualization::initialize()
{
  return true;
}

bool PointCloudVisualization::visualize(const grid_map::GridMap& map)
{
  if (pointCloudPublisher_.getNumSubscribers () < 1) return true;

  sensor_msgs::PointCloud2 pointCloud;

  map.toPointCloud(pointCloud, pointType_);

  pointCloudPublisher_.publish(pointCloud);
  return true;
}

} /* namespace */
