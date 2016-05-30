/*
 * SubmapIteratorSparse.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/SubmapIteratorSparse.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

SubmapIteratorSparse::SubmapIteratorSparse(const grid_map::SubmapGeometry& submap, const int every)
    : SubmapIteratorSparse(submap.getGridMap(), submap.getStartIndex(), submap.getSize(), every)
{
}

SubmapIteratorSparse::SubmapIteratorSparse(const grid_map::GridMap& gridMap,
                               const grid_map::BufferRegion& bufferRegion, const int every)
    : SubmapIteratorSparse(gridMap, bufferRegion.getStartIndex(), bufferRegion.getSize(), every)
{
}


SubmapIteratorSparse::SubmapIteratorSparse(const grid_map::GridMap& gridMap, const Index& submapStartIndex,
                               const Size& submapSize, const int every)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  index_ = submapStartIndex;
  submapSize_ = submapSize;
  submapStartIndex_ = submapStartIndex;
  submapIndex_.setZero();
  isPastEnd_ = false;
  every_ = every;
}

SubmapIteratorSparse::SubmapIteratorSparse(const SubmapIteratorSparse* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  submapSize_ = other->submapSize_;
  submapStartIndex_ = other->submapStartIndex_;
  index_ = other->index_;
  submapIndex_ = other->submapIndex_;
  isPastEnd_ = other->isPastEnd_;
  every_ = other->every_;
}

SubmapIteratorSparse& SubmapIteratorSparse::operator =(const SubmapIteratorSparse& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  submapSize_ = other.submapSize_;
  submapStartIndex_ = other.submapStartIndex_;
  index_ = other.index_;
  submapIndex_ = other.submapIndex_;
  isPastEnd_ = other.isPastEnd_;
  every_ = other.every_;
  return *this;
}

bool SubmapIteratorSparse::operator !=(const SubmapIteratorSparse& other) const
{
  return (index_ != other.index_).any();
}

const Index& SubmapIteratorSparse::operator *() const
{
  return index_;
}

const Index& SubmapIteratorSparse::getSubmapIndex() const
{
  return submapIndex_;
}

SubmapIteratorSparse& SubmapIteratorSparse::operator ++()
{
  isPastEnd_ = !incrementIndexForSubmapSparse(submapIndex_, index_, submapStartIndex_, every_,
                                        submapSize_, size_, startIndex_);
  return *this;
}

bool SubmapIteratorSparse::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */

