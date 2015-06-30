/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/PolygonIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

PolygonIterator::PolygonIterator(const grid_map::GridMap& gridMap, const grid_map::Polygon& polygon)
    : polygon_(polygon)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Eigen::Array2i submapStartIndex;
  Eigen::Array2i submapBufferSize;
  findSubmapParameters(polygon, submapStartIndex, submapBufferSize);
  internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
  if(!isInside()) ++(*this);
}

PolygonIterator& PolygonIterator::operator =(const PolygonIterator& other)
{
  polygon_ = other.polygon_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool PolygonIterator::operator !=(const PolygonIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Eigen::Array2i& PolygonIterator::operator *() const
{
  return *(*internalIterator_);
}

PolygonIterator& PolygonIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return *this;

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) break;
  }

  return *this;
}

bool PolygonIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool PolygonIterator::isInside()
{
  Eigen::Vector2d position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  return polygon_.isInside(position);
}

void PolygonIterator::findSubmapParameters(const grid_map::Polygon& polygon, Eigen::Array2i& startIndex, Eigen::Array2i& bufferSize) const
{
  Eigen::Vector2d topLeft = polygon_.getVertices()[0];
  Eigen::Vector2d bottomRight = topLeft;
  for (const auto& vertex : polygon_.getVertices()) {
    topLeft = topLeft.array().max(vertex.array());
    bottomRight = bottomRight.array().min(vertex.array());
  }
  limitPositionToRange(topLeft, mapLength_, mapPosition_);
  limitPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Eigen::Array2i endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = endIndex - startIndex + Eigen::Array2i::Ones();
}

} /* namespace grid_map */

