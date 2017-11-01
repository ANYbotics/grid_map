/*
 * LineThickIteratorAlt.hpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Perry Franklin
 */

#pragma once


#include "grid_map_core/iterators/FillIterator.hpp"

#include <memory>

namespace grid_map {

class LineThickIteratorAlt {

public:

  LineThickIteratorAlt(const GridMap& grid_map, const Position& start, const Position& end, double thickness);

  ~LineThickIteratorAlt();

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  LineThickIteratorAlt& operator =(const LineThickIteratorAlt& other);

   /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const LineThickIteratorAlt& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  LineThickIteratorAlt& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  const GridMap& map_;

  std::unique_ptr<FillIterator> fill_iterator_;


  bool getIndexLimitedToMapRange(const grid_map::GridMap& gridMap,
                                               const Position& start, const Position& end,
                                               Index& index);


};


}
