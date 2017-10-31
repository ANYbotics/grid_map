/*
 * LineThickIterator.hpp
 *
 *  Created on: Oct 30, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/iterators/LineIteratorAlt.hpp"

#include <memory>

namespace grid_map {

class LineThickIterator {

public:

  LineThickIterator(const GridMap& grid_map, Position start, Position end, double thickness);

  ~LineThickIterator();

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  LineThickIterator& operator =(const LineThickIterator& other);

   /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const LineThickIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  LineThickIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  const GridMap& map_;

  std::unique_ptr<LineIteratorAlt> current_line_;

  const double half_goal_thickness_;

  double half_current_thickness_;

  Position start_;
  Position end_;

  Position perp_step_;
  int perp_index_;

  bool finished_;

};


}
