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

/*!
 * Iterator class to iterate over a thickened line in the map.
 * Uses a FillIterator to iterate through cells within the thickness
 * of the line. Any cell that intersects the thickened line (even if
 * its just a small corner) is iterated over.
 */
class LineThickIteratorAlt {

public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting point of the line.
   * @param end the ending point of the line.
   * @param thickness the thickness of the line.
   */
  LineThickIteratorAlt(const GridMap& grid_map, const Position& start, const Position& end, double thickness);

  /*!
   * Destructor.
   */
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

  /*!
   * Finds an index along the line within the GridMap, if it exists.
   * @param gridMap the map to constrain the line to
   * @param start starting position of the line
   * @param end ending position of the line
   * @param index the return value (the first valid index on the line)
   * @return true if succeeded, false if the line does not cross the GridMap
   */
  bool getIndexLimitedToMapRange(const grid_map::GridMap& gridMap,
                                 const Position& start, const Position& end,
                                 Index& index);


};


}
