/*
 * FillIterator.cpp
 *
 *  Created on: Oct 2, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/iterators/FillIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"
#include <iostream>

using namespace std;

namespace grid_map {

FillIterator::FillIterator(const GridMap& grid_map, const Position& start_location, const IndexChecker& index_checker , bool eight_connected ):
    index_checker_(index_checker.clone()),
    queue_exhausted_(false),
    grid_map_(grid_map),
    traveled_grid_(grid_map_.getSize()),
    eight_connected_(eight_connected)
{

  grid_map_.getIndex(start_location, starting_index_);

  current_index_ = starting_index_;

  // Check to make sure our starting point is actually valid.
  if (!index_checker_->check(starting_index_)){
    queue_exhausted_ = true;
  }
  else
  {
    queue_.push(starting_index_);
    operator++();
  }

}

FillIterator::FillIterator(const GridMap& grid_map, const Index& start_index, const IndexChecker& index_checker, bool eight_connected ):
    index_checker_(index_checker.clone()),
    queue_exhausted_(false),
    grid_map_(grid_map),
    traveled_grid_(grid_map_.getSize()),
    starting_index_(start_index),
    eight_connected_(eight_connected)
{
  current_index_ = starting_index_;

  // Check to make sure our starting point is actually valid.
  if (!index_checker_->check(starting_index_)){
    queue_exhausted_ = true;
  }
  else
  {
    queue_.push(starting_index_);
    operator++();
  }

}

FillIterator::~FillIterator(){
  delete index_checker_;
}

FillIterator& FillIterator::operator =(const FillIterator& other)
{
  current_index_ = other.current_index_;
  traveled_grid_ = other.traveled_grid_;
  queue_ = other.queue_;
  queue_exhausted_ = other.queue_exhausted_;

  std::cout<<"WARNING: The assignment operator is not fully implemented for FillIterator"<<std::endl;

  return *this;
}

bool FillIterator::operator !=(const FillIterator& other) const
{
  return true;
}

const Index& FillIterator::operator *() const
{
  return current_index_;
}

FillIterator& FillIterator::operator ++()
{

  while(true){

    // Verify that the queue is not empty. If it is, we are done.
    if ( queue_.empty() ){
      queue_exhausted_ = true;
      break;
    }
    else
    {
      // Pop off the first index to check.
      const Index temp_index = queue_.front();
      queue_.pop();

      // If we already traveled to this index, we skip it and loop back around.
      if ( !traveled_grid_.checkIfMarked(temp_index)){


        // If we haven't, we're moving to this index.
        current_index_ = temp_index;
        traveled_grid_.mark(temp_index);

        // We try to add the neighbors to the queue.

        //TODO for all 4 neighbors, check if traveled to, and then check if boundary function.
        const Index west_index = current_index_ + Index(1,0);
        tryToAddToQueue(west_index);

        const Index east_index = current_index_ + Index(-1,0);
        tryToAddToQueue(east_index);

        const Index north_index = current_index_ + Index(0,1);
        tryToAddToQueue(north_index);

        const Index south_index = current_index_ + Index(0,-1);
        tryToAddToQueue(south_index);

        if (eight_connected_){
          const Index nwest_index = current_index_ + Index(1,1);
          tryToAddToQueue(nwest_index);

          const Index neast_index = current_index_ + Index(-1,1);
          tryToAddToQueue(neast_index);

          const Index swest_index = current_index_ + Index(1,-1);
          tryToAddToQueue(swest_index);

          const Index seast_index = current_index_ + Index(-1,-1);
          tryToAddToQueue(seast_index);
        }

        break;
      }
    }
  }

  return *this;
}

bool FillIterator::isPastEnd() const
{
  return queue_exhausted_;
}

void FillIterator::tryToAddToQueue(const Index& index){
  if (!checkIfIndexWithinRange(index, grid_map_.getSize())){

  }
  else if (traveled_grid_.checkIfMarked(index)){

  }
  else if (!index_checker_->check( index )){
    traveled_grid_.mark(index);
  }
  else {
    queue_.push(index);
  }

}

FillIterator::MarkerGrid::MarkerGrid(const Size& grid_dimensions):
  grid_dimensions_(grid_dimensions),
  mark_record_(grid_dimensions.x()*grid_dimensions.y(), false){

}

FillIterator::MarkerGrid::MarkerGrid(const MarkerGrid& other):
  grid_dimensions_(other.grid_dimensions_),
  mark_record_(other.mark_record_){
}

bool FillIterator::MarkerGrid::checkIfMarked(const Index& index) const{
  size_t i = index.x()+index.y()*grid_dimensions_.x();
  return mark_record_.at(i);
}

void FillIterator::MarkerGrid::mark(const Index& index){
  size_t i = index.x()+index.y()*grid_dimensions_.x();
   mark_record_.at(i) = true;
}

} // namespace grid_map


