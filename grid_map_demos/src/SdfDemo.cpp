/*
* SdfDemo.cpp
*
*  Created on: May 3, 2022
*      Author: Ruben Grandia
*   Institute: ETH Zurich
 */

#include "grid_map_demos/SdfDemo.hpp"

#include <sensor_msgs/PointCloud2.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace grid_map_demos {

SdfDemo::SdfDemo(ros::NodeHandle& nodeHandle, const std::string& mapTopic, std::string elevationLayer, const std::string& pointcloudTopic)
    : elevationLayer_(std::move(elevationLayer)) {
  pointcloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/full_sdf", 1);
  freespacePublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/free_space", 1);
  occupiedPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/occupied_space", 1);
  gridmapSubscriber_ = nodeHandle.subscribe(mapTopic, 1, &SdfDemo::callback, this);
}

void SdfDemo::callback(const grid_map_msgs::GridMap& message) {
  // Convert message to map.
  grid_map::GridMap map;
  std::vector<std::string> layers{elevationLayer_};
  grid_map::GridMapRosConverter::fromMessage(message, map, layers, false, false);
  auto& elevationData = map.get(elevationLayer_);

  // Inpaint if needed.
  if (elevationData.hasNaN()) {
    const float inpaint{elevationData.minCoeffOfFinites()};
    ROS_WARN("[SdfDemo] Map contains NaN values. Will apply inpainting with min value.");
    elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v)? v : inpaint; });
  }

  // Generate SDF.
  const float heightMargin{0.1};
  const float minValue{elevationData.minCoeffOfFinites() - heightMargin};
  const float maxValue{elevationData.maxCoeffOfFinites() + heightMargin};
  grid_map::SignedDistanceField sdf(map, elevationLayer_, minValue, maxValue);

  // Extract as point clouds.
  sensor_msgs::PointCloud2 pointCloud2Msg;
  grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg);
  pointcloudPublisher_.publish(pointCloud2Msg);

  grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float sdfValue) { return sdfValue > 0.0; });
  freespacePublisher_.publish(pointCloud2Msg);

  grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float sdfValue) { return sdfValue <= 0.0; });
  occupiedPublisher_.publish(pointCloud2Msg);
}

}  // namespace grid_map_demos