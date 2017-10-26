/*
 * NearestValidIterator.cpp
 *
 *  Created on: Oct 25, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/iterators/NearestValidIterator.hpp"

namespace grid_map {

NearestValidIterator::NearestValidIterator(const grid_map::GridMap& gridMap, const Position& start, const IndexChecker&  checker):
 grid_map_(gridMap),
 starting_position_(start),
 index_checker_(checker.clone())
 {
  Index starting_index;

  gridMap.getIndex(start, starting_index);

  spiral_grid_iterator_ = std::unique_ptr<SpiralGridIterator>(new SpiralGridIterator(grid_map_, starting_index)) ;

  operator ++();
}

NearestValidIterator::~NearestValidIterator(){
  delete index_checker_;
}

const Index& NearestValidIterator::operator*() const{

  if (valid_index_queue_.empty()){
    return grid_map_.getStartIndex();
  }

  return valid_index_queue_.top().index;
}

NearestValidIterator& NearestValidIterator::operator ++(){

  if (!valid_index_queue_.empty()){
    valid_index_queue_.pop();
  }

  // This while loop uses the SpiralGridIterator to search for the next nearest point.
  // So if the spiral_grid_iterator is past the end, we've already search all valid points.
  while(!spiral_grid_iterator_->isPastEnd()){
    // The closest distance possible in this ring of the SpiralGridIterator to the starting point
    // is from the edge of the starting index to the inner edge of the ring being searched.
    // This is an lower bound for a starting position anywhere in the starting square.
    const double current_closest_spiral_distance = grid_map_.getResolution()*(spiral_grid_iterator_->getCurrentRadius()-1);

    double current_closest_valid_distance;
    if (!valid_index_queue_.empty()){
      current_closest_valid_distance= valid_index_queue_.top().distance;
    } else {
      current_closest_valid_distance = std::numeric_limits<double>::max();
    }

    // If the closest possible point in this spiral is further away that the current best, we stop searching.
    if (current_closest_spiral_distance > current_closest_valid_distance) {
      break;
    }


    const Index& current_check_index = **spiral_grid_iterator_;

    if (index_checker_->check(current_check_index)){
      // We've found a valid cell.
      DistanceIndexPair di_pair;
      di_pair.index = current_check_index;

      Position index_position;
      grid_map_.getPosition(current_check_index, index_position);

      di_pair.distance = (index_position - starting_position_).norm();

      valid_index_queue_.push(di_pair);
    }

    ++(*spiral_grid_iterator_);

  }

  return *this;
}

bool NearestValidIterator::isPastEnd() const{
  return (valid_index_queue_.empty() && spiral_grid_iterator_->isPastEnd());
}

}  // namespace grid_map


