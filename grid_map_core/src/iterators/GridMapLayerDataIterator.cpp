/*
 * GridMapLayerDataIterator.cpp
 *
 *  Created on: Feb 15, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/GridMapLayerDataIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

GridMapLayerDataIterator::GridMapLayerDataIterator(const grid_map::GridMap& gridMap, const std::string& layer)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  linearSize_ = size_.prod();
  linearIndex_ = 0;
  dataStart_ = gridMap[layer].data();
  isPastEnd_ = false;
}

GridMapLayerDataIterator::GridMapLayerDataIterator(const GridMapLayerDataIterator* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  linearSize_ = other->linearSize_;
  linearIndex_ = other->linearIndex_;
  dataStart_ = other->dataStart_;
  isPastEnd_ = other->isPastEnd_;
}

GridMapLayerDataIterator& GridMapLayerDataIterator::operator =(const GridMapLayerDataIterator& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  linearSize_ = other.linearSize_;
  linearIndex_ = other.linearIndex_;
  dataStart_ = other.dataStart_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool GridMapLayerDataIterator::operator !=(const GridMapLayerDataIterator& other) const
{
  return linearIndex_ != other.linearIndex_;
}

Index GridMapLayerDataIterator::operator *() const
{
  return getIndexFromLinearIndex(linearIndex_, size_);
}

const float& GridMapLayerDataIterator::getValue() const
{
  return *(dataStart_ + (int)linearIndex_);
}

const size_t& GridMapLayerDataIterator::getLinearIndex() const
{
  return linearIndex_;
}

const Index GridMapLayerDataIterator::getUnwrappedIndex() const
{
  return getIndexFromBufferIndex(*(*this), size_, startIndex_);
}

GridMapLayerDataIterator& GridMapLayerDataIterator::operator ++()
{
  size_t newIndex = linearIndex_ + 1;
  if (newIndex < linearSize_) {
    linearIndex_ = newIndex;
  } else {
    isPastEnd_ = true;
  }
  return *this;
}

GridMapLayerDataIterator GridMapLayerDataIterator::end() const
{
  GridMapLayerDataIterator res(this);
  res.linearIndex_ = linearSize_ - 1;
  return res;
}

bool GridMapLayerDataIterator::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */
