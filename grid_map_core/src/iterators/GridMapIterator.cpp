/*
 * GridMapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

GridMapIterator::GridMapIterator(const grid_map::GridMap& gridMap)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  endIndex_ = startIndex_ + gridMap.getSize() - Eigen::Array2i::Ones();
  mapIndexWithinRange(endIndex_, size_);
  index_ = startIndex_;
  isPastEnd_ = false;
}

GridMapIterator::GridMapIterator(const GridMapIterator* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  endIndex_ = other->endIndex_;
  index_ = other->index_;
  isPastEnd_ = other->isPastEnd_;
}

GridMapIterator& GridMapIterator::operator =(const GridMapIterator& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  endIndex_ = other.endIndex_;
  index_ = other.index_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool GridMapIterator::operator !=(const GridMapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Index& GridMapIterator::operator *() const
{
  return index_;
}

const Index GridMapIterator::getUnwrappedIndex() const
{
  return getIndexFromBufferIndex(index_, size_, startIndex_);
}

GridMapIterator& GridMapIterator::operator ++()
{
  isPastEnd_ = !incrementIndex(index_, size_, startIndex_);
  return *this;
}

GridMapIterator GridMapIterator::end() const
{
  GridMapIterator res(this);
  res.index_ = endIndex_;
  return res;
}

bool GridMapIterator::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */
