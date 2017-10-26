/*
 * IndexCheckerNan.hpp
 *
 *  Created on: Oct 16, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/IndexChecker.hpp"

namespace grid_map {

/*!
 * The IndexCheckerNan class implements the IndexChecker.
 * It checks if indices in a layer of a GridMap are NaN.
 */
class IndexCheckerNan : public IndexChecker {

public:

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   * @param layer the layer of the GridMap to check.
   */
  IndexCheckerNan(const GridMap& map, const std::string& layer);

  /*!
   * Checks if GridMap at index is NaN.
   * @param index the index to check.
   */
  bool check(const Index& index) const override;

  IndexChecker* clone() const override;

private:

  const std::string layer_;

};

}  // namespace grid_map
