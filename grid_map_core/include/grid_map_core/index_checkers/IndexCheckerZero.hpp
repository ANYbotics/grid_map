/*
 * IndexCheckerZero.hpp
 *
 *  Created on: Oct 16, 2017
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
class IndexCheckerZero : public IndexChecker {

public:

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   * @param layer the layer of the GridMap to check.
   */
  IndexCheckerZero(const GridMap& map, const std::string& layer);

  /*!
   * Checks if GridMap at index is zero.
   * @param index the index to check.
   */
  bool check(const Index& index) const override;

  IndexChecker* clone() const override;

private:

  const std::string layer_;

};

}  // namespace grid_map
