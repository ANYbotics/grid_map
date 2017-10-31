/*
 * LineIteratorAlt.hpp
 *
 *  Created on: Oct 30, 2017
 *      Author: Perry Franklin
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
class LineIteratorAlt
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting point of the line.
   * @param end the ending point of the line.
   */
  LineIteratorAlt(const grid_map::GridMap& gridMap, const Position& start, const Position& end, bool only_true_line = false);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   */
  LineIteratorAlt(const grid_map::GridMap& gridMap, const Index& start, const Index& end);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  LineIteratorAlt& operator =(const LineIteratorAlt& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const LineIteratorAlt& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  LineIteratorAlt& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:


  /*!
   * Construct function.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   * @return true if successful, false otherwise.
   */
  bool initialize(const grid_map::GridMap& gridMap, const Position& start, const Position& end);

  /*!
   * Computes the parameters requires for the line drawing algorithm.
   */
  void initializeIterationParameters();

  /*!
   * Finds the index of a position on a line within the limits of the map.
   * @param[in] gridMap the grid map that defines the map boundaries.
   * @param[in] start the position that will be limited to the map range.
   * @param[in] end the ending position of the line.
   * @param[out] index the index of the moved start position.
   * @return true if successful, false otherwise.
   */
  bool getIndexLimitedToMapRange(const grid_map::GridMap& gridMap, const Position& start,
                                 const Position& end, Index& index);

  const grid_map::GridMap& map_;

  //! Current index.
  Index index_;

  //! Starting index of the line.
  Position start_;
  Index start_index_;

  //! Ending index of the line.
  Position end_;
  Index end_index_;

  //! Current cell number.
  unsigned int iCell_;

  //! Number of cells in the line.
  unsigned int nCells_;

  //! Helper variables for Bresenham Line Drawing algorithm.
  Size increment1_, increment2_;
  double error_, error_add_;

  //! Map information needed to get position from iterator.
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;
  Index bufferStartIndex_;

  bool only_true_line_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */
