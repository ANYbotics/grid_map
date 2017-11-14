/*
 * IndexCheckerNonZero.hpp
 *
 *  Created on: Oct 14, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/index_checkers/IndexChecker.hpp"

namespace grid_map {

/*!
 * The IndexCheckerNonZero class implements the IndexChecker.
 * It checks if an index in a layer of a GridMap is not zero
 * (ie it returns true if the value is nonzero).
 */
class IndexCheckerNonZero : public IndexChecker {

public:

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   * @param layer the layer of the GridMap to check.
   */
  IndexCheckerNonZero(const GridMap& map, const std::string& layer);

  /*!
   * Checks if GridMap at index is nonzero.
   * @param index the index to check.
   */
  bool check(const Index& index) const override;

  std::unique_ptr<IndexChecker> clone() const override;

private:

  const std::string layer_;

};

}  // namespace grid_map
