/*
 * IndexChecker.hpp
 *
 *  Created on: Oct 4, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/GridMap.hpp"

#include <memory>

namespace grid_map{

/*!
 * The IndexChecker provides an interface for checking an index on a GridMap.
 * It is intended to give a "yes/no" answer for some query on a single space,
 * such as whether a particular layer's value is below a threshold.
 */
class IndexChecker{

public:

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   */
  IndexChecker(const GridMap& grid_map):map_(grid_map){}

  /*!
   * Destructor.
   */
  virtual ~IndexChecker(){}

  /*!
   * Checks the index.
   * @param index the index to check.
   */
  virtual bool check(const Index& index) const = 0;

  /*!
   * Constructs a new copy of the derived IndexChecker.
   * Note that the new IndexChecker is allocated on the heap,
   * and so will need to be deleted.
   */
  virtual std::unique_ptr<IndexChecker> clone() const = 0;

protected:

  const GridMap& map_;

};

}  //  namespace grid_map
