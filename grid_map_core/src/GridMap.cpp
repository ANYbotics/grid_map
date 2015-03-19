/*
 * GridMap.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/GridMapMath.hpp"

// STL
#include <iostream>
#include <cassert>
#include <math.h>
#include <algorithm>
#include <stdexcept>

using namespace std;
using namespace Eigen;

namespace grid_map_core {

GridMap::GridMap(const std::vector<std::string>& types)
{
  position_.setZero();
  length_.setZero();
  resolution_ = 0.0;
  size_.setZero();
  startIndex_.setZero();
  timestamp_ = 0;
  types_ = types;

  for (auto& type : types_) {
    data_.insert(std::pair<std::string, MatrixXf>(type, MatrixXf()));
  }
}

GridMap::GridMap() :
    GridMap(std::vector<std::string>())
{
}

GridMap::~GridMap()
{
}

void GridMap::setGeometry(const Eigen::Array2d& length, const double resolution, const Eigen::Vector2d& position)
{
  assert(length(0) > 0.0);
  assert(length(1) > 0.0);
  assert(resolution > 0.0);

  Array2i size;
  size(0) = static_cast<int>(round(length(0) / resolution)); // There is no round() function in Eigen.
  size(1) = static_cast<int>(round(length(1) / resolution));
  resize(size);
  clearAll();

  resolution_ = resolution;
  length_ = (size_.cast<double>() * resolution_).matrix();
  position_ = position;
  startIndex_.setZero();

  return;
}

void GridMap::setBasicTypes(const std::vector<std::string>& basicTypes)
{
  basicTypes_ = basicTypes;
}

void GridMap::add(const std::string& type)
{
  add(type, Eigen::MatrixXf::Constant(size_(0), size_(1), NAN));
}

void GridMap::add(const std::string& type, const Matrix& data)
{
  assert(size_(0) == data.rows());
  assert(size_(1) == data.cols());

  if (exists(type)) {
    // Type exists already, overwrite its data.
    data_.at(type) = data;
  } else {
    // Type does not exist yet, add type and data.
    data_.insert(std::pair<std::string, MatrixXf>(type, data));
    types_.push_back(type);
  }
}

bool GridMap::exists(const std::string& type) const
{
  return !(data_.find(type) == data_.end());
}

const Eigen::MatrixXf& GridMap::get(const std::string& type) const
{
  try {
    return data_.at(type);
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::get(...) : No map layer of type '" + type + "' available.");
  }
}

Eigen::MatrixXf& GridMap::get(const std::string& type)
{
  try {
    return data_.at(type);
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::get(...) : No map layer of type '" + type + "' available.");
  }
}

bool GridMap::remove(const std::string& type)
{
  const auto dataIterator = data_.find(type);
  if (dataIterator == data_.end()) return false;
  data_.erase(dataIterator);

  const auto typePosition = std::find(types_.begin(), types_.end(), type);
  if (typePosition == types_.end()) return false;
  types_.erase(typePosition);

  const auto basicTypePosition = std::find(basicTypes_.begin(), basicTypes_.end(), type);
  if (basicTypePosition != basicTypes_.end()) basicTypes_.erase(basicTypePosition);

  return true;
}

float& GridMap::atPosition(const std::string& type, const Eigen::Vector2d& position)
{
  Eigen::Array2i index;
  if (getIndex(position, index)) {
    return at(type, index);
  }
  throw std::out_of_range("GridMap::atPosition(...) : Position is out of range.");
}

float GridMap::atPosition(const std::string& type, const Eigen::Vector2d& position) const
{
  Eigen::Array2i index;
  if (getIndex(position, index)) {
    return at(type, index);
  }
  throw std::out_of_range("GridMap::atPosition(...) : Position is out of range.");
}

float& GridMap::at(const std::string& type, const Eigen::Array2i& index)
{
  try {
    return data_.at(type)(index(0), index(1));
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::at(...) : No map layer of type '" + type + "' available.");
  }
}

float GridMap::at(const std::string& type, const Eigen::Array2i& index) const
{
  try {
    return data_.at(type)(index(0), index(1));
  } catch (const std::out_of_range& exception) {
    throw std::out_of_range("GridMap::at(...) : No map layer of type '" + type + "' available.");
  }
}

bool GridMap::getIndex(const Eigen::Vector2d& position, Eigen::Array2i& index) const
{
  return getIndexFromPosition(index, position, length_, position_, resolution_, size_, startIndex_);
}

bool GridMap::getPosition(const Eigen::Array2i& index, Eigen::Vector2d& position) const
{
  return getPositionFromIndex(position, index, length_, position_, resolution_, size_, startIndex_);
}

bool GridMap::isInside(const Eigen::Vector2d& position)
{
  return checkIfPositionWithinMap(position, length_, position_);
}

bool GridMap::isValid(const Eigen::Array2i& index) const
{
  return isValid(index, basicTypes_);
}

bool GridMap::isValid(const Eigen::Array2i& index, const std::string& type) const
{
  if (!isfinite(at(type, index))) return false;
  return true;
}

bool GridMap::isValid(const Eigen::Array2i& index, const std::vector<std::string>& types) const
{
  if (types.empty()) return false;
  for (auto& type : types) {
    if (!isfinite(at(type, index))) return false;
  }
  return true;
}

bool GridMap::getPosition3(const std::string& type, const Index& index, Position3& position) const
{
  if (!isValid(index, type)) return false;
  Vector2d position2d;
  getPosition(index, position2d);
  position.head(2) = position2d;
  position.z() = at(type, index);
  return true;
}

bool GridMap::getVector(const std::string& typePrefix, const Eigen::Array2i& index, Eigen::Vector3d& vector) const
{
  std::vector<std::string> types;
  types.push_back(typePrefix + "x");
  types.push_back(typePrefix + "y");
  types.push_back(typePrefix + "z");
  if (!isValid(index, types)) return false;
  for (size_t i = 0; i < 3; ++i) {
    vector(i) = at(types[i], index);
  }
  return true;
}

GridMap GridMap::getSubmap(const Position& position, const Length& length, bool& isSuccess)
{
  Index index;
  return getSubmap(position, length, isSuccess);
}

GridMap GridMap::getSubmap(const Position& position, const Length& length, Index& indexInSubmap, bool& isSuccess)
{
  // Submap the generate.
  GridMap submap(types_);
  submap.setBasicTypes(basicTypes_);
  submap.setTimestamp(timestamp_);
  submap.setFrameId(frameId_);

  // Get submap geometric information.
  Array2i topLeftIndex;
  Array2i submapBufferSize;
  Eigen::Vector2d submapPosition;
  Array2d submapLength;

  if (!getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, indexInSubmap, position,
                       length, length_, position_, resolution_, size_, startIndex_)) {
    isSuccess = false;
    return GridMap(types_);
  }

  submap.setGeometry(submapLength, resolution_, submapPosition);
  submap.startIndex_.setZero(); // Because of the way we copy the data below.

  // Copy data.
  std::vector<Eigen::Array2i> submapIndeces;
  std::vector<Eigen::Array2i> submapSizes;

  if (!getBufferRegionsForSubmap(submapIndeces, submapSizes, topLeftIndex, submap.size_, size_, startIndex_)) {
    cout << "Cannot access submap of this size." << endl;
    isSuccess = false;
    return GridMap(types_);
  }

  for (auto& data : data_) {
    submap.data_[data.first].topLeftCorner    (submapSizes[0](0), submapSizes[0](1)) = data.second.block(submapIndeces[0](0), submapIndeces[0](1), submapSizes[0](0), submapSizes[0](1));
    submap.data_[data.first].topRightCorner   (submapSizes[1](0), submapSizes[1](1)) = data.second.block(submapIndeces[1](0), submapIndeces[1](1), submapSizes[1](0), submapSizes[1](1));
    submap.data_[data.first].bottomLeftCorner (submapSizes[2](0), submapSizes[2](1)) = data.second.block(submapIndeces[2](0), submapIndeces[2](1), submapSizes[2](0), submapSizes[2](1));
    submap.data_[data.first].bottomRightCorner(submapSizes[3](0), submapSizes[3](1)) = data.second.block(submapIndeces[3](0), submapIndeces[3](1), submapSizes[3](0), submapSizes[3](1));
  }

  isSuccess = true;
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
    if (indexShift(i) != 0) {
      if (abs(indexShift(i)) >= getSize()(i)) {
        // Entire map is dropped.
        clear();
      } else {
        // Drop cells out of map.
        int sign = (indexShift(i) > 0 ? 1 : -1);
        int startIndex = startIndex_(i) - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift(i);
        int nCells = abs(indexShift(i));

        int index = (sign > 0 ? startIndex : endIndex);
        mapIndexWithinRange(index, getSize()(i));

        if (index + nCells <= getSize()(i)) {
          // One region to drop.
          if (i == 0) clearCols(index, nCells);
          if (i == 1) clearRows(index, nCells);
        } else {
          // Two regions to drop.
          int firstIndex = index;
          int firstNCells = getSize()(i) - firstIndex;
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
  startIndex_ += indexShift;
  mapIndexWithinRange(startIndex_, getSize());
  position_ += alignedPositionShift;

//  if (indexShift.all() != 0) // TODO Move notifier to implementation.
//    ROS_DEBUG("Grid map has been moved to position (%f, %f).", position_.x(), position_.y());
}

void GridMap::setTimestamp(const uint64_t& timestamp)
{
  timestamp_ = timestamp;
}

const uint64_t& GridMap::getTimestamp() const
{
  return timestamp_;
}

void GridMap::resetTimestamp()
{
  timestamp_ = 0.0;
}

void GridMap::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

const std::string& GridMap::getFrameId() const
{
  return frameId_;
}

const Eigen::Array2d& GridMap::getLength() const
{
  return length_;
}

const Eigen::Vector2d& GridMap::getPosition() const
{
  return position_;
}

double GridMap::getResolution() const
{
  return resolution_;
}

const grid_map_core::Size& GridMap::getSize() const
{
  return size_;
}

const Eigen::Array2i& GridMap::getStartIndex() const
{
  return startIndex_;
}

void GridMap::clear()
{
  if (basicTypes_.empty()) {
    clearAll();
    return;
  }
  for (auto& key : basicTypes_) {
    data_.at(key).setConstant(NAN);
  }
}

void GridMap::clearAll()
{
  std::cout << "clear all " << std::endl;
  for (auto& data : data_) {
    data.second.setConstant(NAN);
  }
}

void GridMap::clearCols(unsigned int index, unsigned int nCols)
{
  for (auto& type : basicTypes_) {
    data_.at(type).block(index, 0, nCols, getSize()(1)).setConstant(NAN);
  }
}

void GridMap::clearRows(unsigned int index, unsigned int nRows)
{
  for (auto& type : basicTypes_) {
    data_.at(type).block(0, index, getSize()(0), nRows).setConstant(NAN);
  }
}

void GridMap::resize(const Eigen::Array2i& size)
{
  size_ = size;
  for (auto& data : data_) {
    data.second.resize(size_(0), size_(1));
  }
}

} /* namespace */

