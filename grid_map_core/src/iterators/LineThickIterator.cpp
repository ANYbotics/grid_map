/*
 * LineThickIterator.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/iterators/LineThickIterator.hpp"

namespace grid_map{

LineThickIterator::LineThickIterator(const GridMap& grid_map, Position start, Position end, double thickness):
  map_(grid_map),
  start_(start),
  end_(end),
  half_goal_thickness_(thickness/2.0),
  half_current_thickness_(0.0),
  perp_index_(0),
  finished_(false){

  current_line_.reset(new LineIteratorAlt(map_, start, end, true));

  if (current_line_->isPastEnd()){
    finished_ = true;
  }

  Position dir = (end-start).normalized();
  Position perp_dir(-(dir.y()), dir.x());
  if (std::abs(perp_dir.x()) > std::abs(perp_dir.y())){
    perp_step_ = map_.getResolution()*perp_dir.x() * perp_dir;
  } else {
    perp_step_ = -map_.getResolution()*perp_dir.y() * perp_dir;
  }

}

LineThickIterator::~LineThickIterator(){

}

const Index& LineThickIterator::operator *() const{
  return **current_line_;
}

LineThickIterator& LineThickIterator::operator ++(){
  current_line_->operator ++();

  //If we're done with this line
  if (current_line_->isPastEnd()){


    if (perp_index_ <= 0){
      perp_index_ *= -1;
      ++perp_index_;
    } else {
      perp_index_ *= -1;
    }

    Position step = perp_index_*perp_step_;

    if (step.norm() > half_goal_thickness_){
      finished_ = true;
    } else {
      Position new_start = start_ + step;
      Position new_end = end_ + step;

      current_line_.reset(new LineIteratorAlt(map_, new_start, new_end, true));
    }

  }

  return *this;
}

bool LineThickIterator::isPastEnd() const{
  return finished_;
}

}  // namespace grid_map


