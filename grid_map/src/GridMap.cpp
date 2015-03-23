/*
 * GridMap.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMap.hpp"
#include "grid_map/GridMapMsgHelpers.hpp"
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>

// STL
#include <limits>
#include <algorithm>

using namespace std;
using namespace Eigen;

namespace grid_map {

GridMap::GridMap()
 : grid_map_core::GridMap()
{

}

GridMap::GridMap(const std::vector<std::string>& layers)
 : grid_map_core::GridMap(layers)
{

}

GridMap::GridMap(const grid_map_msgs::GridMap& message)
 : grid_map_core::GridMap()
{
  fromMessage(message);
}

GridMap::~GridMap()
{

}

GridMap GridMap::getSubmap(const Position& position, const Length& length, bool& isSuccess)
{
  Index index;
  return getSubmap(position, length, index, isSuccess);
}

GridMap GridMap::getSubmap(const Position& position, const Length& length, Index& indexInSubmap, bool& isSuccess)
{
  grid_map_core::GridMap subMap = grid_map_core::GridMap::getSubmap(position, length, indexInSubmap, isSuccess);
  return *static_cast<grid_map::GridMap*>(&subMap);
}

void GridMap::toMessage(grid_map_msgs::GridMap& message) const
{
  toMessage(layers_, message);
}

void GridMap::toMessage(const std::vector<std::string>& layers, grid_map_msgs::GridMap& message) const
{
  message.info.header.stamp.fromNSec(timestamp_);
  message.info.header.frame_id = frameId_;
  message.info.resolution = resolution_;
  message.info.length_x = length_.x();
  message.info.length_y = length_.y();
  message.info.pose.position.x = position_.x();
  message.info.pose.position.y = position_.y();
  message.info.pose.position.z = 0.0;
  message.info.pose.orientation.x = 0.0;
  message.info.pose.orientation.y = 0.0;
  message.info.pose.orientation.z = 0.0;
  message.info.pose.orientation.w = 1.0;

  for (const auto& layer : layers) {
    message.layers.push_back(layer);
    std_msgs::Float32MultiArray dataArray;
    matrixEigenCopyToMultiArrayMessage(data_.at(layer), dataArray);
    message.data.push_back(dataArray);
  }

  message.outer_start_index = startIndex_(0);
  message.inner_start_index = startIndex_(1);
}

bool GridMap::fromMessage(const grid_map_msgs::GridMap& message)
{
  timestamp_ = message.info.header.stamp.toNSec();
  frameId_ = message.info.header.frame_id;
  resolution_ = message.info.resolution;
  length_.x() = message.info.length_x;
  length_.y() = message.info.length_y;
  position_.x() = message.info.pose.position.x;
  position_.y() = message.info.pose.position.y;

  if (message.layers.size() != message.data.size()) {
    ROS_ERROR("Different number of layers and data in grid map message.");
    return false;
  }

  for (unsigned int i = 0; i < message.layers.size(); i++) {
    Eigen::MatrixXf dataMatrix;
    multiArrayMessageCopyToMatrixEigen(message.data[i], dataMatrix); // TODO Could we use the data mapping (instead of copying) method here?
    data_.insert(std::pair<std::string, Matrix>(message.layers[i], dataMatrix));
    layers_.push_back(message.layers[i]);
  }

  basicLayers_ = layers_;
  size_ << getRows(message.data[0]), getCols(message.data[0]);
  startIndex_(0) = message.outer_start_index;
  startIndex_(1) = message.inner_start_index;
  return true;
}

void GridMap::toPointCloud(sensor_msgs::PointCloud2& pointCloud, const std::string& pointLayer) const
{
  toPointCloud(pointCloud, pointLayer, layers_);
}

void GridMap::toPointCloud(sensor_msgs::PointCloud2& pointCloud, const std::string& pointLayer,
                           const std::vector<std::string>& layersToAdd) const
{
  // Header.
  pointCloud.header.frame_id = frameId_;
  pointCloud.header.stamp.fromNSec(timestamp_);
  pointCloud.is_dense = false;

  // Fields.
  std::vector<std::string> fieldNames;

  for (const auto& layer : layersToAdd) {
    if (layer == pointLayer) {
      fieldNames.push_back("x");
      fieldNames.push_back("y");
      fieldNames.push_back("z");
    } else if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  pointCloud.fields.clear();
  pointCloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (auto& name : fieldNames) {
    sensor_msgs::PointField point_field;
    point_field.name = name;
    point_field.count = 1;
    point_field.datatype = sensor_msgs::PointField::FLOAT32;
    point_field.offset = offset;
    pointCloud.fields.push_back(point_field);
    offset = offset + point_field.count * 4; // 4 for sensor_msgs::PointField::FLOAT32
  }

  // Resize.
  size_t nPoints = size_(0) * size_(1);
  pointCloud.height = 1;
  pointCloud.width = nPoints;
  pointCloud.point_step = offset;
  pointCloud.row_step = pointCloud.width * pointCloud.point_step;
  pointCloud.data.resize(pointCloud.height * pointCloud.row_step);

  // Points
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> fieldIterators;
  for (auto& name : fieldNames) {
    fieldIterators.insert(std::pair<std::string, sensor_msgs::PointCloud2Iterator<float>>
                          (name, sensor_msgs::PointCloud2Iterator<float>(pointCloud, name)));
  }

  GridMapIterator mapIterator(*this);

  for (size_t i = 0; i < nPoints; ++i) {
    Position3 position;
    position.setConstant(NAN);
    getPosition3(pointLayer, *mapIterator, position);

    for (auto& iterator : fieldIterators) {
      if (iterator.first == "x") {
        *iterator.second = (float) position.x();
      } else if (iterator.first == "y") {
        *iterator.second = (float) position.y();
      } else if (iterator.first == "z") {
        *iterator.second = (float) position.z();
      } else if (iterator.first == "rgb") {
        *iterator.second = at("color", *mapIterator);;
      } else {
        *iterator.second = at(iterator.first, *mapIterator);
      }
    }

    ++mapIterator;
    for (auto& iterator : fieldIterators) {
      ++iterator.second;
    }
  }
}

void GridMap::toOccupancyGrid(nav_msgs::OccupancyGrid& occupancyGrid, const std::string& layer,
                              float dataMin, float dataMax) const
{
  occupancyGrid.header.frame_id = frameId_;
  occupancyGrid.header.stamp.fromNSec(timestamp_);
  occupancyGrid.info.map_load_time = occupancyGrid.header.stamp; // Same as header stamp as we do not load the map.
  occupancyGrid.info.resolution = resolution_;
  occupancyGrid.info.width = size_(0);
  occupancyGrid.info.height = size_(1);
  Position positionOfOrigin;
  getPositionOfDataStructureOrigin(position_, length_, positionOfOrigin);
  occupancyGrid.info.origin.position.x = positionOfOrigin.x();
  occupancyGrid.info.origin.position.y = positionOfOrigin.y();
  occupancyGrid.info.origin.position.z = 0.0;
  occupancyGrid.info.origin.orientation.x = 0.0;
  occupancyGrid.info.origin.orientation.y = 0.0;
  occupancyGrid.info.origin.orientation.z = 1.0; // yes, this is correct.
  occupancyGrid.info.origin.orientation.w = 0.0;
  occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);
  
//  const float cellMin = numeric_limits<int8_t>::min();
//  const float cellMax = numeric_limits<int8_t>::max();
  // Occupancy probabilities are in the range [0,100].  Unknown is -1.
  const float cellMin = 0;
  const float cellMax = 100;
  const float cellRange = cellMax - cellMin;

  for (GridMapIterator iterator(*this); !iterator.isPassedEnd(); ++iterator) {
    float value = (at(layer, *iterator) - dataMin) / (dataMax - dataMin);
    if (isnan(value)) value = -1;
    else value = cellMin + min(max(0.0f, value), 1.0f) * cellRange;
    // Occupancy grid claims to be row-major order, but it does not seem that way.
    // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html.
    unsigned int index = get1dIndexFrom2dIndex(*iterator, size_, false);
    occupancyGrid.data[index] = value;
  }
}
  
} /* namespace */
