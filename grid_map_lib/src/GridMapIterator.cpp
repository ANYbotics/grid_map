/*
 * GridMapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/GridMapIterator.hpp"
#include "grid_map_lib/GridMapMath.hpp"

namespace grid_map_lib {

GridMapIterator::GridMapIterator(const grid_map_lib::GridMap& gridMap)
{
  bufferSize_ = gridMap.getBufferSize();
  startIndex_ = gridMap.getBufferStartIndex();
  endIndex_ = startIndex_ + gridMap.getBufferSize() - Eigen::Array2i::Ones();
  mapIndexWithinRange(endIndex_, bufferSize_);
  index_ = startIndex_;
  isPassedEnd_ = false;
}

GridMapIterator::GridMapIterator(const GridMapIterator* other)
{
  bufferSize_ = other->bufferSize_;
  startIndex_ = other->startIndex_;
  endIndex_ = other->endIndex_;
  index_ = other->index_;
  isPassedEnd_ = other->isPassedEnd_;
}

GridMapIterator& GridMapIterator::operator =(const GridMapIterator& other)
{
  bufferSize_ = other.bufferSize_;
  startIndex_ = other.startIndex_;
  endIndex_ = other.endIndex_;
  index_ = other.index_;
  isPassedEnd_ = other.isPassedEnd_;
  return *this;
}

bool GridMapIterator::operator !=(const GridMapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Eigen::Array2i& GridMapIterator::operator *() const
{
  return index_;
}

GridMapIterator& GridMapIterator::operator ++()
{
  isPassedEnd_ = !incrementIndex(index_, bufferSize_, startIndex_);
  return *this;
}

GridMapIterator GridMapIterator::end() const
{
  GridMapIterator res(this);
  res.index_ = endIndex_;
  return res;
}

bool GridMapIterator::isPassedEnd() const
{
  return isPassedEnd_;
}

} /* namespace grid_map_lib */
