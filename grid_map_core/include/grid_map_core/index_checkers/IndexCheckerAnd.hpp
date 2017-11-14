/*
 * IndexCheckerAnd.hpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Perry Franklin
 */


#pragma once

#include "grid_map_core/index_checkers/IndexChecker.hpp"

namespace grid_map {

/*!
 * The IndexCheckerAnd class implements the IndexChecker.
 * It checks if all of the checkers passed to it are true;
 * in other words, a logical AND.
 */
class IndexCheckerAnd : public IndexChecker {

public:

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   * @param layer the layer of the GridMap to check.
   */
  IndexCheckerAnd(const GridMap& map);

  /*!
   * Destructor
   */
  virtual ~IndexCheckerAnd();

  /*!
   * Adds a
   */
  void addChecker(const IndexChecker& checker);

  /*!
   * Checks if GridMap at index is NaN.
   * @param index the index to check.
   */
  bool check(const Index& index) const override;

  IndexChecker* clone() const override;

private:

  std::vector<IndexChecker*> checks_;

};

}  // namespace grid_map
