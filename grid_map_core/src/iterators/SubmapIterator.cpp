/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

SubmapIterator::SubmapIterator(const grid_map::GridMap& gridMap,
                               const Eigen::Array2i& submapStartIndex,
                               const Eigen::Array2i& submapSize)
{
  bufferSize_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  index_ = submapStartIndex;
  submapStartIndex_ = submapStartIndex;
  submapEndIndex_ = submapStartIndex + submapSize - Eigen::Array2i::Ones();
  printf("start: %i,%i\n",submapStartIndex_(0),submapStartIndex_(1));
  printf("end: %i,%i\n",submapEndIndex_(0),submapEndIndex_(1));
  limitIndexToRange(submapEndIndex_, bufferSize_);
  // readjust submab Buffer Size to mapped end Index
  printf("start: %i,%i\n",submapStartIndex_(0),submapStartIndex_(1));
  printf("end: %i,%i\n",submapEndIndex_(0),submapEndIndex_(1));
  submapBufferSize_ = submapEndIndex_ - submapStartIndex_ + Eigen::Array2i::Ones();
  // resize the submap here such that it fits my map
  submapIndex_.setZero();
  isPastEnd_ = false;
}

SubmapIterator::SubmapIterator(const SubmapIterator* other)
{
  bufferSize_ = other->bufferSize_;
  startIndex_ = other->startIndex_;
  submapBufferSize_ = other->submapBufferSize_;
  submapStartIndex_ = other->submapStartIndex_;
  submapEndIndex_ = other->submapEndIndex_;
  index_ = other->index_;
  submapIndex_ = other->submapIndex_;
  isPastEnd_ = other->isPastEnd_;
}

SubmapIterator& SubmapIterator::operator =(const SubmapIterator& other)
{
  bufferSize_ = other.bufferSize_;
  startIndex_ = other.startIndex_;
  submapBufferSize_ = other.submapBufferSize_;
  submapStartIndex_ = other.submapStartIndex_;
  submapEndIndex_ = other.submapEndIndex_;
  index_ = other.index_;
  submapIndex_ = other.submapIndex_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool SubmapIterator::operator !=(const SubmapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Eigen::Array2i& SubmapIterator::operator *() const
{
  return index_;
}

const Eigen::Array2i& SubmapIterator::getSubmapIndex() const
{
  return submapIndex_;
}

SubmapIterator& SubmapIterator::operator ++()
{
  isPastEnd_ = !incrementIndexForSubmap(submapIndex_, index_, submapStartIndex_,
                                         submapBufferSize_, bufferSize_, startIndex_);
  return *this;
}

SubmapIterator SubmapIterator::end() const
{
  SubmapIterator res(this);
  res.index_ = submapEndIndex_;
  return res;
}

bool SubmapIterator::isPastEnd() const
{
  return isPastEnd_;
}

const Eigen::Array2i& SubmapIterator::getSubmapSize() const
{
  return submapBufferSize_;
}

} /* namespace grid_map */

