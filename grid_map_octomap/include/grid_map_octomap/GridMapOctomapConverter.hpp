/*
 * GridMapOctomapConverter.hpp
 *
 *  Created on: May 1, 2017
 *      Author: Jeff Delmerico
 *	 Institute: University of Zürich, Robotics and Perception Group
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>

// Octomap
#include <octomap/octomap.h>

// STD
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

namespace grid_map {

/*!
 * Conversions between grid maps and Octomap types.
 */
class GridMapOctomapConverter
{
 public:
  /*!
   * Default constructor.
   */
  GridMapOctomapConverter();

  /*!
   * Destructor.
   */
  virtual ~GridMapOctomapConverter();

  /*!
   * Converts an octomap to a grid map in the same coordinate frame, with a
   * cell resolution equal to the leaf voxel size in the octomap. Only creates
   * a layer for elevation.
   * This changes the geometry of the grid map and deletes all layer contents.
   * @param[in] octomap the octomap
   * @param[out] gridMap the grid map to be initialized
   * @param[in] min_point (optional) minimum coordinate for bounding box
   * @param[in] max_point (optional) maximum coordinate for bounding box
   * @return true if successful, false otherwise
   */
  static bool fromOctomap(const octomap::OcTree& octomap,
                          grid_map::GridMap& gridmap,
                          const grid_map::Position3* min_point = nullptr,
                          const grid_map::Position3* max_point = nullptr);

};

} /* namespace */
