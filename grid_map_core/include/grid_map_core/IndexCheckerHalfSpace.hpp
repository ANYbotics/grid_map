/*
 * IndexCheckerHalfSpace.hpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Perry Franklin
 */


#pragma once

#include "grid_map_core/IndexChecker.hpp"

namespace grid_map {

/*!
 * The IndexCheckerHalfSpace class implements the IndexChecker.
 * It checks an index lies within a halfspace (ie on one side
 * of the plane). Computes in the form of:
 *    A*x < c
 * where A is the normal, c is an offset, and x is a point
 * in the grid cell.
 * Note: if any of the grid cell is within the halfspace, it
 * returns true.
 */
class IndexCheckerHalfSpace : public IndexChecker {

public:

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   * @param normal the normal of the splitting plane (points away from the valid spaces)
   * @param a point on the splitting plane
   */
  IndexCheckerHalfSpace(const GridMap& map, const Vector& normal, double offset);

  /*!
   * Constructor.
   * @param gridMap the GridMap to query.
   * @param normal the normal of the splitting plane (points away from the valid spaces)
   * @param a point on the splitting plane
   */
  IndexCheckerHalfSpace(const GridMap& map, const Vector& normal, const Position& plane_point);

  /*!
   * Checks if GridMap at index is NaN.
   * @param index the index to check.
   */
  bool check(const Index& index) const override;

  IndexChecker* clone() const override;

private:

  Vector normal_;
  double offset_;
  Position cell_corner_offset_;

};

}  // namespace grid_map
