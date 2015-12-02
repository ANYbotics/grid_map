/*
 * EllipseIterator.hpp
 *
 *  Created on: Dec 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/EllipseIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

EllipseIterator::EllipseIterator(const GridMap& gridMap, const Position& center, const Length& length)
    : center_(center),
      length_(length)
{
  lengthSquare_ = length_.square();
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Index submapBufferSize;
  findSubmapParameters(center, length, submapStartIndex, submapBufferSize);
  internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
  if(!isInside()) ++(*this);
}

EllipseIterator& EllipseIterator::operator =(const EllipseIterator& other)
{
  center_ = other.center_;
  length_ = other.length_;
  lengthSquare_ = other.lengthSquare_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool EllipseIterator::operator !=(const EllipseIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Eigen::Array2i& EllipseIterator::operator *() const
{
  return *(*internalIterator_);
}

EllipseIterator& EllipseIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return *this;

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) break;
  }

  return *this;
}

bool EllipseIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool EllipseIterator::isInside()
{
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  double value = ((position - center_).array().square() / lengthSquare_).sum();
  return (value <= 1);
}

void EllipseIterator::findSubmapParameters(const Position& center, const Length& length,
                                           Index& startIndex, Size& bufferSize) const
{
  Position topLeft = center.array() + length;
  Position bottomRight = center.array() - length;
  limitPositionToRange(topLeft, mapLength_, mapPosition_);
  limitPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = endIndex - startIndex + Eigen::Array2i::Ones();
}

} /* namespace grid_map */

