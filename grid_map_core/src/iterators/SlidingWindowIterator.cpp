/*
 * SlidingWindowIterator.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_core/iterators/SlidingWindowIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

#include <iostream>

namespace grid_map {

SlidingWindowIterator::SlidingWindowIterator(const GridMap& gridMap, const std::string& layer,
                                             const size_t windowSize)
    : GridMapIterator(gridMap),
      data_(gridMap[layer])
{
  windowSize_ = windowSize;
  setup(gridMap);
}

SlidingWindowIterator::SlidingWindowIterator(const SlidingWindowIterator* other)
    : GridMapIterator(other),
      data_(other->data_)
{
  windowSize_ = other->windowSize_;
  windowMargin_ = other->windowMargin_;
}

void SlidingWindowIterator::setWindowLength(const GridMap& gridMap, const double windowLength)
{
  windowSize_ = std::round(windowLength / gridMap.getResolution());
  if (windowSize_ % 2 != 1) ++windowSize_;
  setup(gridMap);
}

const Matrix SlidingWindowIterator::getData() const
{
  const Index centerIndex(*(*this));
  Index topLeftIndex(centerIndex - Index(windowMargin_));
  boundIndexToRange(topLeftIndex, size_);
  Index bottomRightIndex(centerIndex + Index(windowMargin_));
  boundIndexToRange(bottomRightIndex, size_);
  Size adjustedWindowSize(bottomRightIndex - topLeftIndex + Size::Ones());
  return data_.block(topLeftIndex(0), topLeftIndex(1), adjustedWindowSize(0), adjustedWindowSize(1));
}

void SlidingWindowIterator::setup(const GridMap& gridMap)
{
  if (!gridMap.isDefaultStartIndex()) throw std::runtime_error(
      "SlidingWindowIterator cannot be used with grid maps that don't have a default buffer start index.");
  if (windowSize_ % 2 == 0) throw std::runtime_error(
      "SlidingWindowIterator has a wrong window size!");
  windowMargin_ = (windowSize_ - 1) / 2;
}

} /* namespace grid_map */
