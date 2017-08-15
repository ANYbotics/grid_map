/*
 * ThickenedLineIterator.hpp
 *
 *  Created on: Aug 15, 2017
 *      Author: Péter Fankhauser, Karl Kangur
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/PolygonIterator.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map {

/*!
 * Iterator class to iterate over a thick line in the map.
 * Based on the PolygonIterator class.
 */
class ThickenedLineIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting position of the line.
   * @param end the ending position of the line.
   * @param thickness the thickness of the line.
   */
  ThickenedLineIterator(const grid_map::GridMap& gridMap, const Position& start, const Position& end, const double thickness);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  ThickenedLineIterator& operator =(const ThickenedLineIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const ThickenedLineIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  ThickenedLineIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:


  /*!
   * Construct function.
   * @param gridMap the grid map to iterate on.
   * @param start the starting position of the line.
   * @param end the ending position of the line.
   * @param thickness the thickness of the line.
   * @return true if successful, false otherwise.
   */
  bool initialize(const grid_map::GridMap& gridMap, const Position& start, const Position& end, const double thickness);

  //! Polygon iterator representing a thick line
  std::shared_ptr<PolygonIterator> polygonIterator_;
};

} /* namespace */
