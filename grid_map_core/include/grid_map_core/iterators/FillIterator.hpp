/*
 * FillIterator.hpp
 *
 *  Created on: Oct 2, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/index_checkers/IndexChecker.hpp"

// vector
#include <memory>
#include <vector>
#include <queue>

namespace grid_map {

/*!
 * Iterator class to iterate through a connected region of the GridMap.
 * This is equivalent to the "fill/paintbucket" operation of image editors.
 */
class FillIterator {
public:

  /*!
   * Constructor.
   * @param grid_map the grid map to iterate on.
   * @param start_location the position to start filling from.
   * @param IndexChecker an IndexChecker that returns true if an index is valid (ie we should iterate over it) and false if not (ie we should not iterate over it).
   * @param eight_connected whether to treat the grid as 8-connected (diagonals are adjacent). Defaults to false
   */
  FillIterator(const GridMap& grid_map, const Position& start_location, const IndexChecker& index_checker, bool eight_connected = false);


  /*!
   * Constructor.
   * @param grid_map the grid map to iterate on.
   * @param start_index the index to start filling from.
   * @param boundry_function a function that returns true if an index is valid (ie we should iterate over it) and false if not (ie we should not iterate over it).
   * @param eight_connected whether to treat the grid as 8-connected (diagonals are adjacent). Defaults to false
   */
  FillIterator(const GridMap& grid_map, const Index& start_index, const IndexChecker& index_checker, bool eight_connected = false);

  /*!
   * Destructor
   */
  ~FillIterator();

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  FillIterator& operator =(const FillIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const FillIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  FillIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  // MarkerGrid is a helper class, which consists of a Grid of bools.
  // This is used to mark grid cells that have already been visited.
  class MarkerGrid {

  public:

    MarkerGrid(const Size& grid_dimensions);
    MarkerGrid(const FillIterator::MarkerGrid& other);

    bool checkIfMarked(const Index& index) const;
    void mark(const Index& index);

  private:
    std::vector<bool> mark_record_;
    Size grid_dimensions_;

  };

  const GridMap& grid_map_;
  Index starting_index_;

  MarkerGrid traveled_grid_;
  std::queue<Index> queue_;
  bool queue_exhausted_;

  Index current_index_;

  const IndexChecker* index_checker_;

  void tryToAddToQueue(const Index& index);

  const bool eight_connected_;

};

} // namespace grid_map
