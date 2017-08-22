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
                                             const EdgeHandling& edgeHandling, const size_t windowSize)
    : GridMapIterator(gridMap),
      edgeHandling_(edgeHandling),
      data_(gridMap[layer])
{
  std::cerr << "SlidingWindowIterator::SlidingWindowIterator() 1" << std::endl;
  windowSize_ = windowSize;
  setup(gridMap);
  std::cerr << "SlidingWindowIterator::SlidingWindowIterator() 2" << std::endl;
}

SlidingWindowIterator::SlidingWindowIterator(const SlidingWindowIterator* other)
    : GridMapIterator(other),
      edgeHandling_(other->edgeHandling_),
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

SlidingWindowIterator& SlidingWindowIterator::operator ++()
{
  if (edgeHandling_ == EdgeHandling::INSIDE) {
    while (!isPastEnd()) {
      GridMapIterator::operator++();
      if (dataInsideMap()) break;
    }
  } else {
    GridMapIterator::operator++();
  }
  return *this;
}

const Matrix SlidingWindowIterator::getData() const
{
  std::cerr << "SlidingWindowIterator::getData() 1" << std::endl;
  const Index centerIndex(*(*this));
  const Index originalTopLeftIndex(centerIndex - Index(windowMargin_));
  std::cerr << "SlidingWindowIterator::getData() 2" << std::endl;
  Index topLeftIndex(originalTopLeftIndex);
  boundIndexToRange(topLeftIndex, size_);
  std::cerr << "SlidingWindowIterator::getData() 3" << std::endl;
  Index bottomRightIndex(centerIndex + Index(windowMargin_));
  boundIndexToRange(bottomRightIndex, size_);
  std::cerr << "SlidingWindowIterator::getData() 4" << std::endl;
  Size adjustedWindowSize(bottomRightIndex - topLeftIndex + Size::Ones());

  std::cerr << "SlidingWindowIterator::getData() 5" << std::endl;
  switch (edgeHandling_) {
    case EdgeHandling::INSIDE:
    case EdgeHandling::CROP:
      std::cerr << "SlidingWindowIterator::getData() 6" << std::endl;
      return data_.block(topLeftIndex(0), topLeftIndex(1), adjustedWindowSize(0), adjustedWindowSize(1));
    case EdgeHandling::EMPTY:
    case EdgeHandling::MEAN:
      const Matrix data = data_.block(topLeftIndex(0), topLeftIndex(1), adjustedWindowSize(0), adjustedWindowSize(1));
      std::cerr << "SlidingWindowIterator::getData() 7" << std::endl;
      Matrix returnData(windowSize_, windowSize_);
      if (edgeHandling_ == EdgeHandling::EMPTY) returnData.setConstant(NAN);
      else if (edgeHandling_ == EdgeHandling::MEAN) returnData.setConstant(data.meanOfFinites());
      const Index topLeftIndexShift(topLeftIndex - originalTopLeftIndex);
      returnData.block(topLeftIndexShift(0), topLeftIndexShift(1), adjustedWindowSize(0), adjustedWindowSize(1)) =
          data_.block(topLeftIndex(0), topLeftIndex(1), adjustedWindowSize(0), adjustedWindowSize(1));
      return returnData;
  }
  return Matrix::Zero(0, 0);
}

void SlidingWindowIterator::setup(const GridMap& gridMap)
{
  if (!gridMap.isDefaultStartIndex()) throw std::runtime_error(
      "SlidingWindowIterator cannot be used with grid maps that don't have a default buffer start index.");
  if (windowSize_ % 2 == 0) throw std::runtime_error(
      "SlidingWindowIterator has a wrong window size!");
  std::cerr << "SlidingWindowIterator::setup() 1" << std::endl;
  windowMargin_ = (windowSize_ - 1) / 2;
  std::cerr << "SlidingWindowIterator::setup() 2" << std::endl;

  if (edgeHandling_ == EdgeHandling::INSIDE) {
    if (!dataInsideMap()) {
      operator++();
    }
  }
  std::cerr << "SlidingWindowIterator::setup() 3" << std::endl;
}

bool SlidingWindowIterator::dataInsideMap() const
{
  const Index centerIndex(*(*this));
  std::cerr << "SlidingWindowIterator::dataInsideMap() 1" << std::endl;
  const Index topLeftIndex(centerIndex - Index(windowMargin_));
  std::cerr << "SlidingWindowIterator::dataInsideMap() 2" << std::endl;
  const Index bottomRightIndex(centerIndex + Index(windowMargin_));
  std::cerr << "SlidingWindowIterator::dataInsideMap() 3" << std::endl;
  return checkIfIndexInRange(topLeftIndex, size_) && checkIfIndexInRange(bottomRightIndex, size_);
}

} /* namespace grid_map */
