/*
 * GridMapOctomapConverter.hpp
 *
 *  Created on: May 1, 2017
 *      Author: Jeff Delmerico, Peter Fankhauser
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
   * Converts an Octomap to a grid map in the same coordinate frame, with a
   * cell resolution equal to the leaf voxel size in the Octomap. Only creates
   * a layer for elevation.
   * This changes the geometry of the grid map and deletes all layer contents.
   * Note: Bounding box coordinates are not checked for sanity - if you provide
   * values outside of the gridmap, undefined behavior may result.
   * @param[in] octomap the octomap.
   * @param[in] layer the layer that is filled with the octomap data.
   * @param[out] gridMap the grid map to be initialized.
   * @param[in] minPoint (optional) minimum coordinate for bounding box.
   * @param[in] maxPoint (optional) maximum coordinate for bounding box.
   * @return true if successful, false otherwise.
   */
  static bool fromOctomap(const octomap::OcTree& octomap,
                          const std::string& layer,
                          grid_map::GridMap& gridMap,
                          const grid_map::Position3* minPoint = nullptr,
                          const grid_map::Position3* maxPoint = nullptr);

};

} /* namespace */
