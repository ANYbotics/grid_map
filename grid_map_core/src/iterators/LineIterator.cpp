/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/LineIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

LineIterator::LineIterator(const grid_map::GridMap& gridMap, const Eigen::Array2i& start, const Eigen::Array2i& end)
    : start_(start),
      end_(end)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Eigen::Array2i submapStartIndex;
  Eigen::Array2i submapBufferSize;
  initializeParameters();
}

LineIterator::LineIterator(const grid_map::GridMap& gridMap, const Eigen::Vector2d& start, const Eigen::Vector2d& end)
{
  // TODO Implement this constructor with range checking.
}

LineIterator& LineIterator::operator =(const LineIterator& other)
{
  index_ = other.index_;
  start_ = other.start_;
  end_ = other.end_;
  iCell_ = other.iCell_;
  nCells_ = other.nCells_;
  increment1_ = other.increment1_;
  increment2_ = other.increment2_;
  denominator_ = other.denominator_;
  numerator_ = other.numerator_;
  numeratorAdd_ = other.numeratorAdd_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool LineIterator::operator !=(const LineIterator& other) const
{
  return (index_ != other.index_).any();
}

const Eigen::Array2i& LineIterator::operator *() const
{
  return index_;
}

LineIterator& LineIterator::operator ++()
{
  numerator_ += numeratorAdd_;  // Increase the numerator by the top of the fraction
  if (numerator_ >= denominator_) {
    numerator_ -= denominator_;
    index_ += increment1_;
  }
  index_ += increment2_;
  ++iCell_;
  return *this;
}

bool LineIterator::isPastEnd() const
{
  return iCell_ >= nCells_;
}

void LineIterator::initializeParameters()
{
  iCell_ = 0;
  index_ = start_;

  Eigen::Array2i delta = (end_ - start_).abs();

  if (end_.x() >= start_.x()) {
    // x-values increasing.
    increment1_.x() = 1;
    increment2_.x() = 1;
  } else {
    // x-values decreasing.
    increment1_.x() = -1;
    increment2_.x() = -1;
  }

  if (end_.y() >= start_.y()) {
    // y-values increasing.
    increment1_.y() = 1;
    increment2_.y() = 1;
  } else {
    // y-values decreasing.
    increment1_.y() = -1;
    increment2_.y() = -1;
  }

  if (delta.x() >= delta.y()) {
    // There is at least one x-value for every y-value.
    increment1_.x() = 0; // Do not change the x when numerator >= denominator.
    increment2_.y() = 0; // Do not change the y for every iteration.
    denominator_ = delta.x();
    numerator_ = delta.x() / 2;
    numeratorAdd_ = delta.y();
    nCells_ = delta.x() + 1; // There are more x-values than y-values.
  } else {
    // There is at least one y-value for every x-value
    increment2_.x() = 0; // Do not change the x for every iteration.
    increment1_.y() = 0; // Do not change the y when numerator >= denominator.
    denominator_ = delta.y();
    numerator_ = delta.y() / 2;
    numeratorAdd_ = delta.x();
    nCells_ = delta.y() + 1; // There are more y-values than x-values.
  }
}

} /* namespace grid_map */
