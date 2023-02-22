/*
 * naiveSignedDistance.h
 *
 *  Created on: Aug 10, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#pragma once

namespace grid_map {
namespace signed_distance_field {

inline Eigen::Matrix<bool, -1, -1> occupancyAtHeight(const Matrix& elevationMap, float height) {
  Eigen::Matrix<bool, -1, -1> occupancy = elevationMap.unaryExpr([=](float val) { return val > height; });
  return occupancy;
}

inline bool isEqualSdf(const Matrix& sdf0, const Matrix& sdf1, float tol) {
  if (sdf0.cols() != sdf1.cols() || sdf0.rows() != sdf1.rows()) {
    return false;
  }

  for (Eigen::Index col = 0; col < sdf0.cols(); ++col) {
    for (Eigen::Index row = 0; row < sdf0.rows(); ++row) {
      if (sdf0(row, col) == sdf1(row, col) || std::abs(sdf0(row, col) - sdf1(row, col)) < tol) {
        continue;
      } else {
        return false;
      }
    }
  }
  return true;
}

// N^2 naive implementation, for testing purposes
inline Matrix naiveSignedDistanceAtHeight(const Matrix& elevationMap, float height, float resolution) {
  Matrix signedDistance(elevationMap.rows(), elevationMap.cols());

  // For each point
  for (Eigen::Index col = 0; col < elevationMap.cols(); ++col) {
    for (Eigen::Index row = 0; row < elevationMap.rows(); ++row) {
      if (elevationMap(row, col) >= height) {  // point in the SDF is below surface
        signedDistance(row, col) = -INF;
        // find closest open space over all other points
        for (Eigen::Index j = 0; j < elevationMap.cols(); ++j) {
          for (Eigen::Index i = 0; i < elevationMap.rows(); ++i) {
            // Compute distance to free cube at location (i, j)
            const float dx{resolution * pixelBorderDistance(i, row)};
            const float dy{resolution * pixelBorderDistance(j, col)};
            const float dz{std::max(elevationMap(i, j) - height, 0.0F)};
            const float currentSignedDistance{-std::sqrt(dx * dx + dy * dy + dz * dz)};
            signedDistance(row, col) = std::max(signedDistance(row, col), currentSignedDistance);
          }
        }
      } else {  // point in the SDF is above surface
        signedDistance(row, col) = INF;
        // find closest object over all other points
        for (Eigen::Index j = 0; j < elevationMap.cols(); ++j) {
          for (Eigen::Index i = 0; i < elevationMap.rows(); ++i) {
            // Compute distance to occupied cube at location (i, j)
            const float dx{resolution * pixelBorderDistance(i, row)};
            const float dy{resolution * pixelBorderDistance(j, col)};
            const float dz{std::max(height - elevationMap(i, j), 0.0F)};
            const float currentSignedDistance{std::sqrt(dx * dx + dy * dy + dz * dz)};
            signedDistance(row, col) = std::min(signedDistance(row, col), currentSignedDistance);
          }
        }
      }
    }
  }

  return signedDistance;
}

inline Matrix naiveSignedDistanceFromOccupancy(const Eigen::Matrix<bool, -1, -1>& occupancyGrid, float resolution) {
  Matrix signedDistance(occupancyGrid.rows(), occupancyGrid.cols());

  // For each point
  for (Eigen::Index col = 0; col < occupancyGrid.cols(); ++col) {
    for (Eigen::Index row = 0; row < occupancyGrid.rows(); ++row) {
      if (occupancyGrid(row, col)) {  // This point is an obstable
        signedDistance(row, col) = -INF;
        // find closest open space over all other points
        for (Eigen::Index j = 0; j < occupancyGrid.cols(); ++j) {
          for (Eigen::Index i = 0; i < occupancyGrid.rows(); ++i) {
            if (!occupancyGrid(i, j)) {
              const float dx{resolution * pixelBorderDistance(i, row)};
              const float dy{resolution * pixelBorderDistance(j, col)};
              const float currentSignedDistance{-std::sqrt(dx * dx + dy * dy)};
              signedDistance(row, col) = std::max(signedDistance(row, col), currentSignedDistance);
            }
          }
        }
      } else {  // This point is in free space
        signedDistance(row, col) = INF;
        // find closest object over all other points
        for (Eigen::Index j = 0; j < occupancyGrid.cols(); ++j) {
          for (Eigen::Index i = 0; i < occupancyGrid.rows(); ++i) {
            if (occupancyGrid(i, j)) {
              const float dx{resolution * pixelBorderDistance(i, row)};
              const float dy{resolution * pixelBorderDistance(j, col)};
              const float currentSignedDistance{std::sqrt(dx * dx + dy * dy)};
              signedDistance(row, col) = std::min(signedDistance(row, col), currentSignedDistance);
            }
          }
        }
      }
    }
  }

  return signedDistance;
}

}  // namespace signed_distance_field
}  // namespace grid_map