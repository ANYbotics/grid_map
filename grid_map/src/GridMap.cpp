/*
 * GridMap.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMap.hpp"
#include "grid_map/GridMapMsgHelpers.hpp"

// STL
#include <iostream>
#include <cassert>

using namespace std;
using namespace Eigen;

namespace grid_map {

GridMap::GridMap(const std::vector<std::string>& types)
 : grid_map_lib::GridMap(types)
{

}

GridMap::~GridMap()
{

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

  for (unsigned int i; i < message.dataDefinition.size(); i++) {
    Eigen::MatrixXf dataMatrix;
    multiArrayMessageCopyToMatrixEigen(message.data[i], dataMatrix); // TODO Could we use the mapping method here?
    data_.insert(std::pair<std::string, Eigen::MatrixXf>(message.dataDefinition[i].data, dataMatrix));
    types_.push_back(message.dataDefinition[i].data);
  }

  clearTypes_ = types_;
  bufferSize_ << getRows(message.data[0]), getCols(message.data[0]);
  return true;
}

bool GridMap::fromMessage(const grid_map_msg::GridMapCircularBuffer& message)
{
  bufferStartIndex_(0) = message.outerStartIndex;
  bufferStartIndex_(1) = message.innerStartIndex;
  return fromMessage(message.gridMap);
}

void GridMap::toMessage(grid_map_msg::GridMap& message)
{
  message.info.header.stamp.fromNSec(timestamp_);
  message.info.header.frame_id = frameId_;
  message.info.resolution = resolution_;
  message.info.lengthX = length_.x();
  message.info.lengthY = length_.y();
  message.info.positionX = position_.x();
  message.info.positionY = position_.y();

  for (auto& data : data_) {
    std_msgs::String definition;
    definition.data = data.first;
    message.dataDefinition.push_back(definition);

    std_msgs::Float32MultiArray dataArray;
    matrixEigenCopyToMultiArrayMessage(data.second, dataArray);
    message.data.push_back(dataArray);
  }
}

void GridMap::toMessage(grid_map_msg::GridMapCircularBuffer& message)
{
  message.outerStartIndex = bufferStartIndex_(0);
  message.innerStartIndex = bufferStartIndex_(1);
  toMessage(message.gridMap);
}

} /* namespace */
