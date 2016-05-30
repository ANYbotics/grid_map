/*
 * GridMapIteratorSparse.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/GridMapIteratorSparse.hpp"
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

GridMapIteratorSparse::GridMapIteratorSparse(const grid_map::GridMap& gridMap, const int every)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  linearSize_ = size_.prod();
  linearIndex_ = 0;
  isPastEnd_ = false;
  every_ = every;
}

GridMapIteratorSparse::GridMapIteratorSparse(const GridMapIteratorSparse* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  linearSize_ = other->linearSize_;
  linearIndex_ = other->linearIndex_;
  isPastEnd_ = other->isPastEnd_;
  every_ = other->every_;
}

GridMapIteratorSparse& GridMapIteratorSparse::operator =(const GridMapIteratorSparse& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  linearSize_ = other.linearSize_;
  linearIndex_ = other.linearIndex_;
  isPastEnd_ = other.isPastEnd_;
  every_ = other.every_;
  return *this;
}

bool GridMapIteratorSparse::operator !=(const GridMapIteratorSparse& other) const
{
  return linearIndex_ != other.linearIndex_;
}

const Index GridMapIteratorSparse::operator *() const
{
  return getIndexFromLinearIndex(linearIndex_, size_);
}

const size_t& GridMapIteratorSparse::getLinearIndex() const
{
  return linearIndex_;
}

const Index GridMapIteratorSparse::getUnwrappedIndex() const
{
  return getIndexFromBufferIndex(*(*this), size_, startIndex_);
}

GridMapIteratorSparse& GridMapIteratorSparse::operator ++()
{
  size_t newIndex;
  if (((int)(linearIndex_ + every_)/size_(0)) % every_ == 0) {
	newIndex = linearIndex_ + every_;
  }
  else {
	newIndex = linearIndex_ + (every_-1)*size_(0) + every_ - (linearIndex_ + every_)%size_(0);
  }
  if (newIndex < linearSize_) {
    linearIndex_ = newIndex;
  } else {
    isPastEnd_ = true;
  }
  return *this;
}

GridMapIteratorSparse GridMapIteratorSparse::end() const
{
  GridMapIteratorSparse res(this);
  res.linearIndex_ = linearSize_ - 1;
  return res;
}

bool GridMapIteratorSparse::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */
