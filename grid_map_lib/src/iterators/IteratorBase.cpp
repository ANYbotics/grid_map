/*
 * IteratorBase.hpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/iterators/IteratorBase.hpp"
#include "grid_map_lib/GridMapMath.hpp"

namespace grid_map_lib {

IteratorBase::IteratorBase(const grid_map_lib::GridMap& gridMap)
{
  bufferSize_ = gridMap.getBufferSize();
  startIndex_ = gridMap.getBufferStartIndex();
  endIndex_ = startIndex_ + gridMap.getBufferSize() - Eigen::Array2i::Ones();
  mapIndexWithinRange(endIndex_, bufferSize_);
  index_ = startIndex_;
  isPassedEnd_ = false;
}

IteratorBase::IteratorBase(const IteratorBase* other)
{
  bufferSize_ = other->bufferSize_;
  startIndex_ = other->startIndex_;
  endIndex_ = other->endIndex_;
  index_ = other->index_;
  isPassedEnd_ = other->isPassedEnd_;
}

IteratorBase& IteratorBase::operator =(const IteratorBase& other)
{
  bufferSize_ = other.bufferSize_;
  startIndex_ = other.startIndex_;
  endIndex_ = other.endIndex_;
  index_ = other.index_;
  isPassedEnd_ = other.isPassedEnd_;
  return *this;
}

bool IteratorBase::operator !=(const IteratorBase& other) const
{
  return (index_ != other.index_).any();
}

const Eigen::Array2i& IteratorBase::operator *() const
{
  return index_;
}

IteratorBase& IteratorBase::operator ++()
{
  isPassedEnd_ = !incrementIndex(index_, bufferSize_, startIndex_);
  return *this;
}

IteratorBase IteratorBase::end() const
{
  IteratorBase res(this);
  res.index_ = endIndex_;
  return res;
}

bool IteratorBase::isPassedEnd() const
{
  return isPassedEnd_;
}

} /* namespace grid_map_lib */
