/*
 * SpiralGridIterator.cpp
 *
 *  Created on: Oct 24, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/iterators/SpiralGridIterator.hpp"

#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

SpiralGridIterator::SpiralGridIterator(const grid_map::GridMap& gridMap, const Index& start, const int radius):
  maximum_radius_(radius),
  current_radius_(0),
  current_index_(start),
  step_(0),
  direction_(0,0),
  grid_map_(gridMap)
{

  // Calculate the maximum possible radius based on grid size and starting index
  const Size& grid_size = gridMap.getSize();

  int max_x_radius = std::max(grid_size.x() - start.x(), start.x());
  int max_y_radius = std::max(grid_size.y() - start.y(), start.y());

  int max_grid_radius = std::max(max_x_radius, max_y_radius);

  // Make sure that maximum_radius_ is at most the maximum possible radius
  if (max_grid_radius < maximum_radius_){
    maximum_radius_ = max_grid_radius;
  }

}

SpiralGridIterator& SpiralGridIterator::operator =(const SpiralGridIterator& other){

  maximum_radius_ = other.maximum_radius_;
  current_radius_ = other.current_radius_;

  current_index_ = other.current_index_;

  step_ = other.step_;
  direction_ = other.current_index_;

  return *this;

}

bool SpiralGridIterator::operator !=(const SpiralGridIterator& other) const{
  return (current_index_ != other.current_index_).any();
}

const Index& SpiralGridIterator::operator *() const{
  return current_index_;
}

SpiralGridIterator& SpiralGridIterator::operator ++(){

  // For this method of iterating, the first space is a special case.
  if (current_radius_ == 0){
    current_index_ = current_index_ + Index(-1,1);
    direction_ = Index(1,0);
    ++current_radius_;
    step_ = 0;
    if (!checkIfIndexWithinRange(current_index_, grid_map_.getSize())){
      operator++();
    }
  }
  else {
    // This counts the steps taken in each direction.
    // So in a radius one ring, we step 2 times to the right, 2 times down,
    // 2 times to the left, 1 time up, and then to the next ring.
    // For a radius 2 ring, we step 4 times to the right, etc etc.
    Index temp_index = current_index_;
    do{

      ++step_;
      temp_index += direction_;

      // We're done stepping in this direction.
      if (step_ == 2*current_radius_){

        if (direction_.isApprox(Index(1,0))){
          direction_ = Index(0,-1);
          step_ = 0;
        }
        else if (direction_.isApprox(Index(0,-1))){
          direction_ = Index(-1,0);
          step_ = 0;
        }
        else if (direction_.isApprox(Index(-1,0))){
          direction_ = Index(0,1);
          step_ = 0;
        }
        // If we were heading north, we finished the ring.
        // Also, we are at the starting index of the ring (ie we already hit it).
        else if (direction_.isApprox(Index(0,1))){
          direction_ = Index(1,0);
          temp_index += Index(-1,1);
          step_ = 0;
          ++current_radius_;

          // If the radius is large enough, we are done.
          if (current_radius_ > maximum_radius_ ){
            return *this;
          }
        }

      }

    } while (!checkIfIndexWithinRange(temp_index, grid_map_.getSize()));

    current_index_ = temp_index;
  }

  return *this;

}


bool SpiralGridIterator::isPastEnd() const{
  return current_radius_ > maximum_radius_;
}

int SpiralGridIterator::getCurrentRadius() const{
  return current_radius_;
}

} // namespace grid_map
