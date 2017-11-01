/*
 * LineThickIteratorAlt.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Perry Franklin
 */


#include "grid_map_core/iterators/LineThickIteratorAlt.hpp"

#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_core/IndexCheckerAnd.hpp"
#include "grid_map_core/IndexCheckerHalfSpace.hpp"

#include "grid_map_core/GridMapMath.hpp"

#include <iostream>

using namespace std;

namespace grid_map {

LineThickIteratorAlt::LineThickIteratorAlt(const grid_map::GridMap& gridMap, const Position& start,
    const Position& end, double thickness):
map_(gridMap)
{

  // Construct the index checker

  Vector perp;
  perp.x() = -( end - start ).y();
  perp.y() = ( end - start ).x();

  Vector normal = perp.normalized();

  Position perp_offset = thickness*(normal);

  IndexCheckerAnd total_checker(map_);
  total_checker.addChecker(IndexCheckerHalfSpace(gridMap, start-end, start));
  total_checker.addChecker(IndexCheckerHalfSpace(gridMap, end-start, end));

  total_checker.addChecker(IndexCheckerHalfSpace(gridMap, normal, start+perp_offset));
  total_checker.addChecker(IndexCheckerHalfSpace(gridMap, -normal, start-perp_offset));

  // Now we need to find a starting point.

  Index starting_index = map_.getStartIndex();
  // Trying to find a point along the line...
  if( !getIndexLimitedToMapRange(map_, start, end, starting_index)){

    //Okay, well then we try to find a point in the submap containing our thick line.

    Position point1 = start+perp;
    Position point2 = start-perp;
    Position point3 = end+perp;
    Position point4 = end-perp;

    double y_max = std::max(std::max(point1.y(), point2.y()), std::max(point3.y(),point4.y()));
    double x_max = std::max(std::max(point1.x(), point2.x()), std::max(point3.x(),point4.x()));
    double y_min = std::min(std::min(point1.y(), point2.y()), std::min(point3.y(),point4.y()));
    double x_min = std::min(std::min(point1.x(), point2.x()), std::min(point3.x(),point4.x()));

    Position top_left(x_max,y_max);
    Position bot_right(x_min,y_min);

    limitPositionToRange(top_left, map_.getLength(), map_.getPosition());
    limitPositionToRange(bot_right, map_.getLength(), map_.getPosition());
    map_.getIndex(top_left, starting_index);
    Index end_index;
    map_.getIndex(bot_right, end_index);
    Size buffer_size = getSubmapSizeFromCornerIndeces(starting_index, end_index, map_.getSize(), map_.getStartIndex());

    for( SubmapIterator sub_iterator(map_, starting_index, buffer_size); !sub_iterator.isPastEnd();++sub_iterator){
      if (total_checker.check(*sub_iterator)){
        starting_index = *sub_iterator;
        break;
      }
    }

  }
  // Note that if all this fails, then the starting index stays at map_ start index,
  // which just means the FillIterator starts at isPastEnd();

  fill_iterator_.reset(new FillIterator(map_,starting_index, total_checker, true));

}

LineThickIteratorAlt::~LineThickIteratorAlt(){

}

LineThickIteratorAlt& LineThickIteratorAlt::operator =(const LineThickIteratorAlt& other)
{
  return *this;
}

bool LineThickIteratorAlt::operator !=(const LineThickIteratorAlt& other) const
{
  return (operator *() != other.operator *()).any();
}

const Index& LineThickIteratorAlt::operator *() const
{
  return fill_iterator_->operator *();
}

LineThickIteratorAlt& LineThickIteratorAlt::operator ++()
{

  fill_iterator_->operator ++();

  return *this;
}



bool LineThickIteratorAlt::isPastEnd() const
{
  return fill_iterator_->isPastEnd();
}

bool LineThickIteratorAlt::getIndexLimitedToMapRange(const grid_map::GridMap& gridMap,
                                                     const Position& start, const Position& end,
                                                     Index& index)
{
  Position newStart = start;
  Vector direction = (end - start).normalized();
  while (!gridMap.getIndex(newStart, index)) {
    newStart += (gridMap.getResolution() - std::numeric_limits<double>::epsilon()) * direction;
    if ((end - newStart).norm() < gridMap.getResolution() - std::numeric_limits<double>::epsilon())
      return false;
  }
  return true;
}

} /* namespace grid_map */
