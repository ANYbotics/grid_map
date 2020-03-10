/*
 * IndexCheckerDifference.hpp
 *
 *  Created on: Oct 4, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <grid_map_core/index_checkers/IndexChecker.hpp>

namespace grid_map{

/*!
 * The IndexCheckerDifference class implements the IndexChecker.
 * On construction, it stores the values of a GridMap at a specific index. When
 * used to check other index, the value of the GridMap at the other index is
 * checked against the stored value of the initial index.
 * Note that the GridMap is stored as a reference, while the values of the
 * initial index are stored as a copy.
 */
class IndexCheckerDifference: public IndexChecker{

public:

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   * @param initial_index the index of the map to compare against.
   */
  IndexCheckerDifference(const GridMap& map, Index initial_index);

  /*!
   * Checks if GridMap at index is different from the GridMap at initial_index.
   * @param index the index to check.
   */
  bool check(Index index) const override;

  std::unique_ptr<IndexChecker> clone() const override;

private:

  /*!
   * Clone constructor.
   * Used for directly passing the values used in the check.
   */
  IndexCheckerDifference(const GridMap& map, std::vector<DataType> values);

  std::vector<DataType> values_;

};

}  // namespace grid_map
