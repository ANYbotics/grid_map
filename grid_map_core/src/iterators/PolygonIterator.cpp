/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <memory>

#include "grid_map_core/iterators/PolygonIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

PolygonIterator::PolygonIterator(const grid_map::GridMap& gridMap, const grid_map::Polygon& polygon)
    : polygon_(polygon)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Size submapBufferSize;
  findSubmapParameters(polygon, submapStartIndex, submapBufferSize);
  internalIterator_ = std::make_shared<SubmapIterator>(gridMap, submapStartIndex, submapBufferSize);
  if (!isInside()) {
    ++(*this);
  }
}

bool PolygonIterator::operator !=(const PolygonIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& PolygonIterator::operator *() const
{
  return *(*internalIterator_);
}

PolygonIterator& PolygonIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) {
    return *this;
  }

  for (; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) {
      break;
    }
  }

  return *this;
}

bool PolygonIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool PolygonIterator::isInside() const
{
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  return polygon_.isInside(position);
}

void PolygonIterator::findSubmapParameters(const grid_map::Polygon& /*polygon*/, Index& startIndex, Size& bufferSize) const
{
  Position topLeft = polygon_.getVertices()[0];
  Position bottomRight = topLeft;
  for (const auto& vertex : polygon_.getVertices()) {
    topLeft = topLeft.array().max(vertex.array());
    bottomRight = bottomRight.array().min(vertex.array());
  }
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = getSubmapSizeFromCornerIndices(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

} /* namespace grid_map */

