/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/SubmapIterator.hpp"
#include "grid_map_lib/GridMapMath.hpp"

using namespace std;

namespace grid_map_lib {

SubmapIterator::SubmapIterator(const grid_map_lib::GridMap& gridMap,
                               const Eigen::Array2i& submapStartIndex,
                               const Eigen::Array2i& submapBufferSize)
{
  bufferSize_ = gridMap.getBufferSize();
  startIndex_ = gridMap.getBufferStartIndex();
  index_ = submapStartIndex;
  submapBufferSize_ = submapBufferSize;
  submapStartIndex_ = submapStartIndex;
  submapEndIndex_ = submapStartIndex + submapBufferSize - Eigen::Array2i::Ones();
  mapIndexWithinRange(submapEndIndex_, submapBufferSize_);
  submapIndex_.setZero();
  isPassedEnd_ = false;
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
  isPassedEnd_ = other->isPassedEnd_;
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
  isPassedEnd_ = other.isPassedEnd_;
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
  isPassedEnd_ = !incrementIndexForSubmap(submapIndex_, index_, submapStartIndex_,
                                         submapBufferSize_, bufferSize_, startIndex_);
  return *this;
}

SubmapIterator SubmapIterator::end() const
{
  SubmapIterator res(this);
  res.index_ = submapEndIndex_;
  return res;
}

bool SubmapIterator::isPassedEnd() const
{
  return isPassedEnd_;
}

} /* namespace grid_map_lib */

