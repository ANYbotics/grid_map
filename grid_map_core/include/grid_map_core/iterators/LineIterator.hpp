/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map {

/*!
 * Iterator class to iterate over a line in the map.
 * Based on Bresenham Line Drawing algorithm.
 */
class LineIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   */
  LineIterator(const grid_map::GridMap& gridMap, const Eigen::Array2i& start, const Eigen::Array2i& end);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting point of the line.
   * @param end the ending point of the line.
   */
  LineIterator(const grid_map::GridMap& gridMap, const Eigen::Vector2d& start, const Eigen::Vector2d& end);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  LineIterator& operator =(const LineIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const LineIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Eigen::Array2i& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  LineIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  // TODO
  void initializeParameters();

  //! Current index.
  Eigen::Array2i index_;

  //! Starting index of the line.
  Eigen::Array2i start_;

  //! Ending index of the line.
  Eigen::Array2i end_;

  //! Current cell number.
  unsigned int iCell_;

  //! Number of cells in the line.
  unsigned int nCells_;

  //! Helper variables for Bresenham Line Drawing algorithm.
  Eigen::Array2i increment1_, increment2_;
  int denominator_, numerator_, numeratorAdd_;

  //! Map information needed to get position from iterator.
  Eigen::Array2d mapLength_;
  Eigen::Vector2d mapPosition_;
  double resolution_;
  Eigen::Array2i bufferSize_;
  Eigen::Array2i bufferStartIndex_;
};

} /* namespace */
