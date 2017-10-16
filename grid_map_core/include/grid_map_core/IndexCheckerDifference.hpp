/*
 * IndexCheckerDifference.hpp
 *
 *  Created on: Oct 4, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <grid_map_core/IndexChecker.hpp>

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

  IndexCheckerDifference(const GridMap& map, Index initial_index);

  bool check(Index index) const override;

private:

  std::vector<DataType> values;

};

}  // namespace grid_map
