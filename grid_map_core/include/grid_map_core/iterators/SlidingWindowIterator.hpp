/*
 * SlidingWindowIterator.hpp
 *
 *  Created on: Aug 17, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"

#include "grid_map_core/iterators/GridMapIterator.hpp"

#include <Eigen/Core>

namespace grid_map {

/*!
 * Iterator class to iterate trough the entire grid map with access to a layer's
 * data through a sliding window.
 * Note: This iterator only works with maps with zero start index.
 */
class SlidingWindowIterator : public GridMapIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param layer the layer on which the data is accessed.
   * @param windowSize the size of the moving window in number of cells (has to be an odd number!).
   */
  SlidingWindowIterator(const GridMap& gridMap, const std::string& layer, const size_t windowSize = 3);

  /*!
   * Copy constructor.
   * @param other the object to copy.
   */
  SlidingWindowIterator(const SlidingWindowIterator* other);

  /*!
   * Set the side length of the moving window (in m).
   * @param gridMap the grid map to iterate on.
   * @param windowLength the side length of the window (in m).
   */
  void setWindowLength(const GridMap& gridMap, const double windowLength);

  /*!
   * Return the data of the sliding window.
   * @return the data of the sliding window.
   */
  const Matrix getData() const;

private:
  //! Setup members.
  void setup(const GridMap& gridMap);

  //! Data.
  const Matrix& data_;

  //! Size of the sliding window.
  size_t windowSize_;

  //! Size of the border of the window around the center cell.
  size_t windowMargin_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */
