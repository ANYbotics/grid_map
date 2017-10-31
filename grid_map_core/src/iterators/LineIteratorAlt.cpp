/*
 * LineIteratorAltAlt.cpp
 *
 *  Created on: Oct 30, 2017
 *      Author: Perry Franklin
 */


#include "grid_map_core/iterators/LineIteratorAlt.hpp"
#include "grid_map_core/GridMapMath.hpp"
#include <iostream>
using namespace std;

namespace grid_map {

LineIteratorAlt::LineIteratorAlt(const grid_map::GridMap& gridMap, const Position& start,
                           const Position& end, bool only_true_line):
                               map_(gridMap),
                               only_true_line_(only_true_line)
{
  if (!initialize(gridMap, start, end)){
    cout << "Warning: initialization of LineIteratorAlt failed."<< endl;
    iCell_ = nCells_;
  }
}

LineIteratorAlt& LineIteratorAlt::operator =(const LineIteratorAlt& other)
{
  index_ = other.index_;
  start_ = other.start_;
  end_ = other.end_;
  iCell_ = other.iCell_;
  nCells_ = other.nCells_;
  increment1_ = other.increment1_;
  increment2_ = other.increment2_;
  error_ = other.error_;
  error_add_ = other.error_add_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  only_true_line_ = other.only_true_line_;
  return *this;
}

bool LineIteratorAlt::operator !=(const LineIteratorAlt& other) const
{
  return (index_ != other.index_).any();
}

const Index& LineIteratorAlt::operator *() const
{
  return index_;
}

LineIteratorAlt& LineIteratorAlt::operator ++()
{
  if (!isPastEnd()){

    error_ += error_add_;  // Increase the numerator by the top of the fraction
    if (error_ >= map_.getResolution()/2) {
      error_ -= map_.getResolution();
      index_ += increment1_;
    }
    else if (error_ <= -map_.getResolution()/2) {
      error_ += map_.getResolution();
      index_ += increment1_;
    }
    std::cout<<"error = "<< error_<<"; error_add_ = "<<error_add_<<std::endl;
    index_ += increment2_;
    ++iCell_;

    if (!only_true_line_){
      // If we are on the last cell, make sure we end in the index with the end position.
      if (iCell_ == nCells_-1){
        index_ = end_index_;
      }
    }

  }

  return *this;
}



bool LineIteratorAlt::isPastEnd() const
{
  return iCell_ >= nCells_;
}

bool LineIteratorAlt::initialize(const grid_map::GridMap& gridMap, const Position& start, const Position& end)
{
    start_ = start;
    end_ = end;

    if (!gridMap.getIndex(start_, start_index_) ||
        !gridMap.getIndex(end_, end_index_)){
      return false;
    }

    resolution_ = gridMap.getResolution();
    bufferSize_ = gridMap.getSize();
    bufferStartIndex_ = gridMap.getStartIndex();

    iCell_ = 0;
    index_ = start_index_;

    Size delta = (end_index_ - start_index_).abs();

    if (end_index_.x() >= start_index_.x()) {
      // x-values increasing.
      increment1_.x() = 1;
      increment2_.x() = 1;
    } else {
      // x-values decreasing.
      increment1_.x() = -1;
      increment2_.x() = -1;
    }

    if (end_index_.y() >= start_index_.y()) {
      // y-values increasing.
      increment1_.y() = 1;
      increment2_.y() = 1;
    } else {
      // y-values decreasing.
      increment1_.y() = -1;
      increment2_.y() = -1;
    }

    Position delta_position = end - start;

    if (delta.x() >= delta.y()) {
      // There is at least one x-value for every y-value.
      increment1_.x() = 0; // Do not change the x when |error| >= map_resolution/2.
      increment2_.y() = 0; // Do not change the y for every iteration.

      // We're moving along x, so we always step by map_resolution in the x direction.
      double ratio = gridMap.getResolution()/delta_position.x();
      error_add_ = ratio*delta_position.y();
      // Also, error_add/map_resolution is now the slope of the line

      // The starting error is the y distance between the index position and the line
      // Not the perpendicular distance, the y distance
      Position start_index_position;
      gridMap.getPosition(start_index_, start_index_position);

      double x_difference = start_index_position.x() - start.x();
      double slope = error_add_/gridMap.getResolution();

      error_ = start_index_position.y() - (x_difference*slope + start.y());

      nCells_ = delta.x() + 1; // There are more x-values than y-values.
    } else {
      // There is at least one y-value for every x-value
      increment2_.x() = 0; // Do not change the x for every iteration.
      increment1_.y() = 0; // Do not change the y when |error| >= 0.5.

      // We're moving along x, so we always step by map_resolution in the y direction.
      double ratio = gridMap.getResolution()/delta_position.y();
      error_add_ = ratio*delta_position.x();
      // Also, error_add/map_resolution is now the slope of the line

      // The starting error is the y distance between the index position and the line
      // Not the perpendicular distance, the y distance
      Position start_index_position;
      gridMap.getPosition(start_index_, start_index_position);

      double y_difference = start_index_position.y() - start.y();
      double slope = error_add_/gridMap.getResolution();

      error_ = start_index_position.x() - (y_difference*slope + start.x());
      nCells_ = delta.y() + 1; // There are more y-values than x-values.
    }

    // If we only want cells in the "infinite" line, then if the error_ starts as too much
    // then we need move to a valid cell.
    if (only_true_line_){
      if (error_ > gridMap.getResolution()/2 ){
        index_ -= increment1_;
        error_-= map_.getResolution();
      } else if (error_ < -gridMap.getResolution()/2 ){
        index_ -= increment1_;
        error_+= map_.getResolution();
      }
    }
    return true;
}

} /* namespace grid_map */



