/*
 * SpiralGridIterator.hpp
 *
 *  Created on: Oct 24, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/GridMap.hpp"

namespace grid_map {

class SpiralGridIterator {

public:

  SpiralGridIterator(const grid_map::GridMap& gridMap, const Index& start, const int radius = std::numeric_limits<int>::max());

  ~SpiralGridIterator();

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  SpiralGridIterator& operator =(const SpiralGridIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const SpiralGridIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  SpiralGridIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

  int getCurrentRadius() const;

private:

  int maximum_radius_;
  int current_radius_;

  Index current_index_;

  int step_;
  Index direction_;

  const grid_map::GridMap& grid_map_;

};

} // namespace grid_map
