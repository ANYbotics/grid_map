/*
 * PolzgonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/PolygonIterator.hpp"
#include "grid_map_lib/GridMapMath.hpp"

using namespace std;

namespace grid_map_lib {

PolygonIterator::PolygonIterator(const grid_map_lib::GridMap& gridMap, const Polygon& polygon)
    : polygon_(polygon),
      internalIterator_(gridMap)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getBufferSize();
  bufferStartIndex_ = gridMap.getBufferStartIndex();
  if(!inPolygon()) ++(*this);
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
  return *internalIterator_;
}

PolygonIterator& PolygonIterator::operator ++()
{
  ++internalIterator_;
  if (internalIterator_.isPassedEnd()) return *this;

  for ( ; !internalIterator_.isPassedEnd(); ++internalIterator_) {
    if (inPolygon()) break;
  }

  return *this;
}

bool PolygonIterator::isPassedEnd() const
{
  return internalIterator_.isPassedEnd();
}

bool PolygonIterator::inPolygon()
{
  Eigen::Vector2d position;
  getPositionFromIndex(position, *internalIterator_, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  return pointInPolygon(position);
}

bool PolygonIterator::pointInPolygon(const Eigen::Vector2d& point)
{
  int cross = 0;
  for (int i = 0, j = polygon_.size() - 1; i < polygon_.size(); j = i++) {
    if ( ((polygon_[i].y() > point.y()) != (polygon_[j].y() > point.y()))
           && (point.x() < (polygon_[j].x() - polygon_[i].x()) * (point.y() - polygon_[i].y()) /
            (polygon_[j].y() - polygon_[i].y()) + polygon_[i].x()) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}

} /* namespace grid_map_lib */

