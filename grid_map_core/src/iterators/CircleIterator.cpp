/*
 * CircleIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/iterators/CircleIterator.hpp"

#include <memory>
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

CircleIterator::CircleIterator(const GridMap& gridMap, const Position& center, const double radius)
    : center_(center),
      radius_(radius)
{
  radiusSquare_ = pow(radius_, 2);
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Index submapBufferSize;
  findSubmapParameters(center, radius, submapStartIndex, submapBufferSize);
  internalIterator_ = std::make_shared<SubmapIterator>(gridMap, submapStartIndex, submapBufferSize);
  if(!isInside()) {
    ++(*this);
  }
}

bool CircleIterator::operator !=(const CircleIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& CircleIterator::operator *() const
{
  return *(*internalIterator_);
}

CircleIterator& CircleIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) {
    return *this;
  }

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) {
      break;
    }
  }

  return *this;
}

bool CircleIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool CircleIterator::isInside() const
{
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  double squareNorm = (position - center_).array().square().sum();
  return (squareNorm <= radiusSquare_);
}

void CircleIterator::findSubmapParameters(const Position& center, const double radius,
                                          Index& startIndex, Size& bufferSize) const
{
  Position topLeft = center.array() + radius;
  Position bottomRight = center.array() - radius;
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = getSubmapSizeFromCornerIndices(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

} /* namespace grid_map */

