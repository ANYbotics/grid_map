/*
 * NearestValidIterator.hpp
 *
 *  Created on: Oct 25, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/GridMap.hpp"

#include "grid_map_core/iterators/SpiralGridIterator.hpp"
#include "grid_map_core/index_checkers/IndexChecker.hpp"

#include <queue>
#include <memory>

namespace grid_map {

/*!
 * Iterator class that visits "valid" cells (defined by an IndexChecker)
 * in the order of nearest to farthest (in relations to the start point)
 */
class NearestValidIterator {

public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting position we wish to find nearest neighbors for.
   * @param checker an Indexchecker than indicates if a cell is valid (and we should iterate over it)
   */
  NearestValidIterator(const grid_map::GridMap& gridMap, const Position& start, const IndexChecker&  checker);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index we wish to find nearest neighbors for.
   * @param checker an Indexchecker than indicates if a cell is valid (and we should iterate over it)
   */
  NearestValidIterator(const grid_map::GridMap& gridMap, const Index& start, const IndexChecker&  checker);

  /*!
   * Destructor
   */
  ~NearestValidIterator();

  /*!
     * Assignment operator.
     * @param iterator the iterator to copy data from.
     * @return a reference to *this.
     */
  NearestValidIterator& operator =(const NearestValidIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const NearestValidIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  NearestValidIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;


private:

  struct DistanceIndexPair {

    double distance;
    Index index;

    bool operator>(const DistanceIndexPair& other) const{
      return distance > other.distance;
    }
  };

  std::priority_queue< DistanceIndexPair, std::vector<DistanceIndexPair>, std::greater<DistanceIndexPair> > valid_index_queue_;

  std::unique_ptr<SpiralGridIterator> spiral_grid_iterator_;

  const std::unique_ptr<IndexChecker> index_checker_;

  const grid_map::GridMap& grid_map_;

  Position starting_position_;
};

} // namespace grid_map
