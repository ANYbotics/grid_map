/*
 * Gridmap3dLookup.h
 *
 *  Created on: Aug 13, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#pragma once

#include <grid_map_core/TypeDefs.hpp>

namespace grid_map {
namespace signed_distance_field {

/**
 * Stores 3 dimensional grid information and provides methods to convert between position - 3d Index - linear index.
 *
 * As with the 2D GridMap, the X-Y position is opposite to the row-col-index: (X,Y) is highest at (0,0) and lowest at (n, m).
 * The z-position is increasing with the layer-index.
 */
struct Gridmap3dLookup {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// size_t in 3 dimensions
  struct size_t_3d {
    size_t x{0};
    size_t y{0};
    size_t z{0};

    size_t_3d() = default;
    size_t_3d(size_t x, size_t y, size_t z) : x(x), y(y), z(z) {}
  };

  //! 3D size of the grid
  size_t_3d gridsize_{0, 0, 0};

  //! Origin position of the grid
  Position3 gridOrigin_{0.0, 0.0, 0.0};

  //! Maximum index per dimension stored as double
  Position3 gridMaxIndexAsDouble_{0.0, 0.0, 0.0};

  //! Grid resolution
  double resolution_{1.0};

  /** Default constructor: creates an empty grid */
  Gridmap3dLookup() = default;

  /**
   * Constructor
   * @param gridsize : x, y, z size of the grid
   * @param gridOrigin : position at x=y=z=0
   * @param resolution : (>0.0) size of 1 voxel
   */
  Gridmap3dLookup(const size_t_3d& gridsize, const Position3& gridOrigin, double resolution)
      : gridsize_(gridsize),
        gridOrigin_(gridOrigin),
        gridMaxIndexAsDouble_(static_cast<double>(gridsize_.x - 1), static_cast<double>(gridsize_.y - 1),
                              static_cast<double>(gridsize_.z - 1)),
        resolution_(resolution) {
    assert(resolution_ > 0.0);
    assert(gridsize_.x > 0);
    assert(gridsize_.y > 0);
    assert(gridsize_.z > 0);
  };

  /** Returns the 3d index of the grid node closest to the query position */
  size_t_3d nearestNode(const Position3& position) const noexcept {
    const double resInv{1.0 / resolution_};
    Position3 subpixelVector{(gridOrigin_.x() - position.x()) * resInv, (gridOrigin_.y() - position.y()) * resInv,
                             (position.z() - gridOrigin_.z()) * resInv};
    return {getNearestPositiveInteger(subpixelVector.x(), gridMaxIndexAsDouble_.x()),
            getNearestPositiveInteger(subpixelVector.y(), gridMaxIndexAsDouble_.y()),
            getNearestPositiveInteger(subpixelVector.z(), gridMaxIndexAsDouble_.z())};
  }

  /** Returns the 3d node position from a 3d index */
  Position3 nodePosition(const size_t_3d& index) const noexcept {
    assert(index.x < gridsize_.x);
    assert(index.y < gridsize_.y);
    assert(index.z < gridsize_.z);
    return {gridOrigin_.x() - index.x * resolution_, gridOrigin_.y() - index.y * resolution_, gridOrigin_.z() + index.z * resolution_};
  }

  /** Returns the linear node index from a 3d node index */
  size_t linearIndex(const size_t_3d& index) const noexcept {
    assert(index.x < gridsize_.x);
    assert(index.y < gridsize_.y);
    assert(index.z < gridsize_.z);
    return (index.z * gridsize_.y + index.y) * gridsize_.x + index.x;
  }

  /** Linear size */
  size_t linearSize() const noexcept { return gridsize_.x * gridsize_.y * gridsize_.z; }

  /** rounds subindex value and clamps it to [0, max] */
  static size_t getNearestPositiveInteger(double val, double max) noexcept {
    // Comparing bounds as double prevents underflow/overflow of size_t
    return static_cast<size_t>(std::max(0.0, std::min(std::round(val), max)));
  }
};

}  // namespace signed_distance_field
}  // namespace grid_map