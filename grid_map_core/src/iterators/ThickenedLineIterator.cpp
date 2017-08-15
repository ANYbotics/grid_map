/*
 * ThickenedLineIterator.cpp
 *
 *  Created on: Aug 15, 2017
 *      Author: Péter Fankhauser, Karl Kangur
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/ThickenedLineIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"
#include "grid_map_core/Polygon.hpp"

using namespace std;

namespace grid_map {

ThickenedLineIterator::ThickenedLineIterator(const grid_map::GridMap& gridMap, const Position& start,
                           const Position& end, const double thickness)
{
  initialize(gridMap, start, end, thickness);
}

ThickenedLineIterator& ThickenedLineIterator::operator =(const ThickenedLineIterator& other)
{
  polygonIterator_ = other.polygonIterator_;
  return *this;
}

bool ThickenedLineIterator::operator !=(const ThickenedLineIterator& other) const
{
  return (polygonIterator_ != other.polygonIterator_);
}

const Index& ThickenedLineIterator::operator *() const
{
  return *(*polygonIterator_);
}

ThickenedLineIterator& ThickenedLineIterator::operator ++()
{
  ++(*polygonIterator_);
  return *this;
}

bool ThickenedLineIterator::isPastEnd() const
{
  return polygonIterator_->isPastEnd();
}

bool ThickenedLineIterator::initialize(const grid_map::GridMap& gridMap, const Position& start, const Position& end, const double thickness)
{
    // Line direction
    Position dir = end - start;

    // Perpendicular to direction
    Position perp = Position(dir.x(), -dir.y());

    // Normalized perpendicular
    Position nperp = perp.normalized();

    // The perpendicular offset, function of the line thickness
    Position perpoffset = nperp * thickness * 0.5;

    // Wrap the line in a Polygon
    Polygon thickLine;
    thickLine.addVertex(start + perpoffset);
    thickLine.addVertex(start - perpoffset);
    thickLine.addVertex(end - perpoffset);
    thickLine.addVertex(end + perpoffset);

    polygonIterator_ = std::shared_ptr<PolygonIterator>(new PolygonIterator(gridMap, thickLine));

    return true;
}

} /* namespace grid_map */
