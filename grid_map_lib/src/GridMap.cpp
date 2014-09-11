/*
 * GridMap.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/GridMap.hpp"
#include "grid_map_lib/GridMapMath.hpp"

// STL
#include <iostream>
#include <cassert>

using namespace std;
using namespace Eigen;

namespace grid_map_lib {

GridMap::GridMap(const std::vector<std::string>& types)
{
  position_.setZero();
  length_.setZero();
  resolution_ = 0.0;
  bufferSize_.setZero();
  bufferStartIndex_.setZero();
  timestamp_ = 0;
  types_ = types;
  clearTypes_ = types_;

  for (auto& type : types_) {
    data_.insert(std::pair<std::string, MatrixXf>(type, MatrixXf()));
  }
}

GridMap::~GridMap()
{

}

void GridMap::setGeometry(const Eigen::Array2d& length, const double resolution, const Eigen::Vector2d& position)
{
  assert(length(0) > 0.0);
  assert(length(1) > 0.0);
  assert(resolution > 0.0);

  Array2i bufferSize;
  bufferSize(0) = static_cast<int>(round(length(0) / resolution)); // There is no round() function in Eigen.
  bufferSize(1) = static_cast<int>(round(length(1) / resolution));
  resizeBuffer(bufferSize);
  clearAll();

  resolution_ = resolution;
  length_ = (bufferSize_.cast<double>() * resolution_).matrix();
  position_ = position;
  bufferStartIndex_.setZero();

  cout << "Grid map matrix resized to " << bufferSize_(0) << " rows and "  << bufferSize_(1) << " columns." << endl;
  return;
}

void GridMap::setClearTypes(const std::vector<std::string>& clearTypes)
{
  clearTypes_ = clearTypes;
}

const Eigen::MatrixXf& GridMap::getConst(const std::string& type)
{
  return data_[type];
}

Eigen::MatrixXf& GridMap::get(const std::string& type)
{
  return data_[type];
}

float& GridMap::at(const std::string& type, const Eigen::Vector2d& position)
{
  Eigen::Array2i index;
  if (getIndex(position, index)) {
    return at(type, index);
  }
}

float& GridMap::at(const std::string& type, const Eigen::Array2i& index)
{
  return data_[type](index(0), index(1));
}

bool GridMap::getIndex(const Eigen::Vector2d& position, Eigen::Array2i& index)
{
  return getIndexFromPosition(index, position, length_, position_, resolution_, bufferSize_, bufferStartIndex_);
}

bool GridMap::getPosition(const Eigen::Array2i& index, Eigen::Vector2d& position)
{
  return getPositionFromIndex(position, index, length_, position_, resolution_, bufferSize_, bufferStartIndex_);
}

bool GridMap::isInside(const Eigen::Vector2d& position)
{
  return checkIfPositionWithinMap(position, length_, position_);
}

bool GridMap::isValid(const Eigen::Array2i& index)
{
  for (auto& type : clearTypes_) {
    if (isnanf(at(type, index))) return false;
  }
  return true;
}

bool GridMap::getPosition3d(const std::string& type, const Eigen::Array2i& index, Eigen::Vector3d& position)
{
  if (!isValid(index)) return false;
  Vector2d position2d;
  getPosition(index, position2d);
  position.head(2) = position2d;
  position.z() = data_[type](index(0), index(1));
  return true;
}

GridMap GridMap::getSubmap(const Eigen::Vector2d& position, const Eigen::Array2d& length, Eigen::Array2i& indexInSubmap, bool& isSuccess)
{
  // Submap the generate.
  GridMap submap(types_);
  submap.setClearTypes(clearTypes_);
//  submap.setHeader(header_); // TODO

  // Get submap geometric information.
  Array2i topLeftIndex;
  Array2i submapBufferSize;
  Eigen::Vector2d submapPosition;
  Array2d submapLength;

  getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, indexInSubmap, position,
                       length, length_, position_, resolution_, bufferSize_, bufferStartIndex_);

  submap.setGeometry(submapLength, resolution_, submapPosition);

  // Copy data.
  std::vector<Eigen::Array2i> submapIndeces;
  std::vector<Eigen::Array2i> submapSizes;

  if (!getBufferRegionsForSubmap(submapIndeces, submapSizes, topLeftIndex, submap.bufferSize_, bufferSize_, bufferStartIndex_)) {
    cout << "Cannot access submap of this size." << endl;
    isSuccess = false;
    return GridMap(types_);
  }

//  Array2i submapBufferSize;
//  submapBufferSize[0] =
//      (submapSizes[3](0) + submapSizes[1](0) > submapSizes[2](0) + submapSizes[0](0)) ?
//          submapSizes[3](0) + submapSizes[1](0) : submapSizes[2](0) + submapSizes[0](0);
//  submapBufferSize[1] =
//      (submapSizes[3](1) + submapSizes[2](1) > submapSizes[1](1) + submapSizes[0](1)) ?
//          submapSizes[3](1) + submapSizes[2](1) : submapSizes[1](1) + submapSizes[0](1);

//  submap.resizeBuffer(submapBufferSize);

  for (auto& data : data_)
  {
    submap.data_[data.first].topLeftCorner    (submapSizes[0](0), submapSizes[0](1)) = data.second.block(submapIndeces[0](0), submapIndeces[0](1), submapSizes[0](0), submapSizes[0](1));
    submap.data_[data.first].topRightCorner   (submapSizes[1](0), submapSizes[1](1)) = data.second.block(submapIndeces[1](0), submapIndeces[1](1), submapSizes[1](0), submapSizes[1](1));
    submap.data_[data.first].bottomLeftCorner (submapSizes[2](0), submapSizes[2](1)) = data.second.block(submapIndeces[2](0), submapIndeces[2](1), submapSizes[2](0), submapSizes[2](1));
    submap.data_[data.first].bottomRightCorner(submapSizes[3](0), submapSizes[3](1)) = data.second.block(submapIndeces[3](0), submapIndeces[3](1), submapSizes[3](0), submapSizes[3](1));
  }

  // Copy other header information.
//  submap.setFrame(frameId_, pose_);
//  submap.setTimeStamp(timeStamp_);

  return submap;
}


void GridMap::move(const Eigen::Vector2d& position)
{
  Array2i indexShift;
  Vector2d positionShift = position - position_;
  getIndexShiftFromPositionShift(indexShift, positionShift, resolution_);
  Vector2d alignedPositionShift;
  getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution_);

  // Delete fields that fall out of map (and become empty cells).
  for (int i = 0; i < indexShift.size(); i++) {
    if (indexShift[i] != 0) {
      if (abs(indexShift[i]) >= getBufferSize()(i)) {
        // Entire map is dropped.
        clear();
      } else {
        // Drop cells out of map.
        int sign = (indexShift[i] > 0 ? 1 : -1);
        int startIndex = bufferStartIndex_[i] - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift[i];
        int nCells = abs(indexShift[i]);

        int index = (sign > 0 ? startIndex : endIndex);
        mapIndexWithinRange(index, getBufferSize()[i]);

        if (index + nCells <= getBufferSize()[i]) {
          // One region to drop.
          if (i == 0) clearCols(index, nCells);
          if (i == 1) clearRows(index, nCells);
        } else {
          // Two regions to drop.
          int firstIndex = index;
          int firstNCells = getBufferSize()[i] - firstIndex;
          if (i == 0) clearCols(firstIndex, firstNCells);
          if (i == 1) clearRows(firstIndex, firstNCells);

          int secondIndex = 0;
          int secondNCells = nCells - firstNCells;
          if (i == 0) clearCols(secondIndex, secondNCells);
          if (i == 1) clearRows(secondIndex, secondNCells);
        }
      }
    }
  }

  // Update information.
  bufferStartIndex_ += indexShift;
  mapIndexWithinRange(bufferStartIndex_, getBufferSize());
  position_ += alignedPositionShift;

//  if (indexShift.all() != 0)
//    ROS_DEBUG("Grid map has been moved to position (%f, %f).", position_.x(), position_.y());
}

void GridMap::setTimestamp(const uint64_t& timestamp)
{
  timestamp_ = timestamp;
}

const uint64_t& GridMap::getTimestamp()
{
  return timestamp_;
}

void GridMap::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

const std::string& GridMap::getFrameId()
{
  return frameId_;
}

const Eigen::Array2d& GridMap::getLength()
{
  return length_;
}

const Eigen::Vector2d& GridMap::getPosition()
{
  return position_;
}

double GridMap::getResolution()
{
  return resolution_;
}

const Eigen::Array2i& GridMap::getBufferSize()
{
  return bufferSize_;
}

const Eigen::Array2i& GridMap::getBufferStartIndex()
{
  return bufferStartIndex_;
}

void GridMap::clear()
{
  for (auto& key : clearTypes_) {
    data_[key].setConstant(NAN);
  }
}

void GridMap::clearAll()
{
  for (auto& data : data_) {
    data.second.setConstant(NAN);
  }
}

void GridMap::clearCols(unsigned int index, unsigned int nCols)
{
  for (auto& type : clearTypes_)
  {
    data_[type].block(index, 0, nCols, getBufferSize()[1]).setConstant(NAN);
  }
}

void GridMap::clearRows(unsigned int index, unsigned int nRows)
{
  for (auto& type : clearTypes_)
  {
    data_[type].block(0, index, getBufferSize()[0], nRows).setConstant(NAN);
  }
}

void GridMap::resizeBuffer(const Eigen::Array2i& bufferSize)
{
  bufferSize_ = bufferSize;

  for (auto& data : data_)
  {
    data.second.resize(bufferSize_(0), bufferSize_(1));
  }
}

} /* namespace */

