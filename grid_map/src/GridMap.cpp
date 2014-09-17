/*
 * GridMap.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMap.hpp"
#include "grid_map/GridMapMsgHelpers.hpp"
#include <grid_map_lib/GridMapIterator.hpp>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace Eigen;

namespace grid_map {

GridMap::GridMap(const std::vector<std::string>& types)
 : grid_map_lib::GridMap(types)
{

}

GridMap::GridMap(const grid_map_msg::GridMap& message)
    : grid_map_lib::GridMap()
{
  fromMessage(message);
}

GridMap::~GridMap()
{

}

GridMap GridMap::getSubmap(const Eigen::Vector2d& position, const Eigen::Array2d& length, Eigen::Array2i& indexInSubmap, bool& isSuccess)
{
  grid_map_lib::GridMap subMap = grid_map_lib::GridMap::getSubmap(position, length, indexInSubmap, isSuccess);
  return *static_cast<grid_map::GridMap*>(&subMap);
}

void GridMap::toMessage(grid_map_msg::GridMap& message) const
{
  toMessage(types_, message);
}

void GridMap::toMessage(const std::vector<std::string>& types, grid_map_msg::GridMap& message) const
{
  message.info.header.stamp.fromNSec(timestamp_);
  message.info.header.frame_id = frameId_;
  message.info.resolution = resolution_;
  message.info.lengthX = length_.x();
  message.info.lengthY = length_.y();
  message.info.positionX = position_.x();
  message.info.positionY = position_.y();

  for (auto& type : types) {
    std_msgs::String definition;
    definition.data = type;
    message.dataDefinition.push_back(definition);

    std_msgs::Float32MultiArray dataArray;
    matrixEigenCopyToMultiArrayMessage(data_.at(type), dataArray);
    message.data.push_back(dataArray);
  }

  message.outerStartIndex = bufferStartIndex_(0);
  message.innerStartIndex = bufferStartIndex_(1);
}

bool GridMap::fromMessage(const grid_map_msg::GridMap& message)
{
  timestamp_ = message.info.header.stamp.toNSec();
  frameId_ = message.info.header.frame_id;
  resolution_ = message.info.resolution;
  length_.x() = message.info.lengthX;
  length_.y() = message.info.lengthY;
  position_.x() = message.info.positionX;
  position_.y() = message.info.positionY;

  if (message.dataDefinition.size() != message.data.size()) {
    ROS_ERROR("Different number of data definition and data in grid map message.");
    return false;
  }

  for (unsigned int i = 0; i < message.dataDefinition.size(); i++) {
    Eigen::MatrixXf dataMatrix;
    multiArrayMessageCopyToMatrixEigen(message.data[i], dataMatrix); // TODO Could we use the data mapping (instead of copying) method here?
    data_.insert(std::pair<std::string, Eigen::MatrixXf>(message.dataDefinition[i].data, dataMatrix));
    types_.push_back(message.dataDefinition[i].data);
  }

  clearTypes_ = types_;
  bufferSize_ << getRows(message.data[0]), getCols(message.data[0]);
  bufferStartIndex_(0) = message.outerStartIndex;
  bufferStartIndex_(1) = message.innerStartIndex;
  return true;
}

void GridMap::toPointCloud(sensor_msgs::PointCloud2& pointCloud, const std::string& pointType) const
{
  toPointCloud(pointCloud, pointType, types_);
}

void GridMap::toPointCloud(sensor_msgs::PointCloud2& pointCloud, const std::string& pointType,
                           const std::vector<std::string>& typesToAdd) const
{
  // Header.
  pointCloud.header.frame_id = frameId_;
  pointCloud.header.stamp.fromNSec(timestamp_);
  pointCloud.is_dense = false;

  // Fields.
  std::vector<std::string> fieldNames;

  for (const auto& type : typesToAdd) {
    if (type == pointType) {
      fieldNames.push_back("x");
      fieldNames.push_back("y");
      fieldNames.push_back("z");
    } else if (type == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(type);
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
  size_t nPoints = bufferSize_(0) * bufferSize_(1);
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

  grid_map_lib::GridMapIterator mapIterator(*this);

  for (size_t i = 0; i < nPoints; ++i) {
    Eigen::Vector3d position;
    position.setConstant(NAN);
    getPosition3d(pointType, *mapIterator, position);

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

} /* namespace */
