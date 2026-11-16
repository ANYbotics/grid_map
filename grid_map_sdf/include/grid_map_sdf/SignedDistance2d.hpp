/*
 * SignedDistance2d.h
 *
 *  Created on: Jul 10, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#pragma once

#include <vector>

#include <grid_map_core/TypeDefs.hpp>

#include "Utils.hpp"

namespace grid_map {
namespace signed_distance_field {

/**
 * Computes the signed distance field at a specified height for a given elevation map.
 *
 * @param elevationMap : elevation data.
 * @param height : height to generate the signed distance at.
 * @param resolution : resolution of the elevation map. (The true distance [m] between cells in world frame)
 * @param minHeight : the lowest height contained in elevationMap
 * @param maxHeight : the maximum height contained in elevationMap
 * @return The signed distance field at the query height.
 */
Matrix signedDistanceAtHeight(const Matrix& elevationMap, float height, float resolution, float minHeight, float maxHeight);

/**
 * Same as above, but returns the sdf in transposed form.
 * Also takes temporary variables from outside to prevent memory allocation.
 *
 * @param elevationMap : elevation data.
 * @param sdfTranspose : [output] resulting sdf in transposed form (automatically allocated if of wrong size)
 * @param tmp : temporary of size elevationMap (automatically allocated if of wrong size)
 * @param tmpTranspose : temporary of size elevationMap transpose (automatically allocated if of wrong size)
 * @param height : height to generate the signed distance at.
 * @param resolution : resolution of the elevation map. (The true distance [m] between cells in world frame)
 * @param minHeight : the lowest height contained in elevationMap
 * @param maxHeight : the maximum height contained in elevationMap
 */
void signedDistanceAtHeightTranspose(const Matrix& elevationMap, Matrix& sdfTranspose, Matrix& tmp, Matrix& tmpTranspose, float height,
                                     float resolution, float minHeight, float maxHeight);

/**
 * Gets the 2D signed distance from an occupancy grid.
 * Returns +INF if there are no obstacles, and -INF if there are only obstacles
 *
 * @param occupancyGrid : occupancy grid with true = obstacle, false = free space
 * @param resolution : resolution of the grid.
 * @return signed distance for each point in the grid to the occupancy border.
 */
Matrix signedDistanceFromOccupancy(const Eigen::Matrix<bool, -1, -1>& occupancyGrid, float resolution);

}  // namespace signed_distance_field
}  // namespace grid_map