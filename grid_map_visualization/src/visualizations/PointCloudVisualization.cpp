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

PointCloudVisualization::PointCloudVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
    : VisualizationBase(nodeHandle, name)
{
}

PointCloudVisualization::~PointCloudVisualization()
{
}

bool PointCloudVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);
  if (!getParam("layer", layer_)) {
    ROS_ERROR("PointCloudVisualization with name '%s' did not find a 'layer' parameter.", name_.c_str());
    return false;
  }
  return true;
}

bool PointCloudVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(name_, 1, true);
  return true;
}

bool PointCloudVisualization::visualize(const grid_map::GridMap& map)
{
  if (publisher_.getNumSubscribers () < 1) return true;
  sensor_msgs::PointCloud2 pointCloud;
  map.toPointCloud(pointCloud, layer_);
  publisher_.publish(pointCloud);
  return true;
}

} /* namespace */
