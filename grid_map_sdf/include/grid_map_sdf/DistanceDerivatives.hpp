/*
 * DistanceDerivatives.h
 *
 *  Created on: Aug 10, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#pragma once

#include <grid_map_core/TypeDefs.hpp>

namespace grid_map {
namespace signed_distance_field {

/**
 * Takes the columnwise central difference of a matrix. Uses single sided difference at the boundaries.
 * diff = (col(i+1) - col(i-1)) / (2 * resolution)
 *
 * @param data [in] : data to take the difference of.
 * @param centralDifference [out] : matrix to store the result in.
 * @param resolution [in] : scaling of the distance between two columns.
 */
inline void columnwiseCentralDifference(const Matrix& data, Matrix& centralDifference, float resolution) {
  assert(data.cols() >= 2);  // Minimum size to take finite differences.

  const Eigen::Index m{data.cols()};
  const float resInv{1.0F / resolution};
  const float doubleResInv{1.0F / (2.0F * resolution)};

  // First column
  centralDifference.col(0) = resInv * (data.col(1) - data.col(0));

  // All the middle columns
  for (Eigen::Index i = 1; i + 1 < m; ++i) {
    centralDifference.col(i) = doubleResInv * (data.col(i + 1) - data.col(i - 1));
  }

  // Last column
  centralDifference.col(m - 1) = resInv * (data.col(m - 1) - data.col(m - 2));
}

/**
 * Takes the finite difference between layers
 * result = (data_{k+1} - data{k}) / resolution
 */
inline void layerFiniteDifference(const Matrix& data_k, const Matrix& data_kp1, Matrix& result, float resolution) {
  const float resInv{1.0F / resolution};
  result = resInv * (data_kp1 - data_k);
}

/**
 * Takes the central difference between layers
 * result = (data_{k+1} - data{k-1}) / (2.0 * resolution)
 */
inline void layerCentralDifference(const Matrix& data_km1, const Matrix& data_kp1, Matrix& result, float resolution) {
  const float doubleResInv{1.0F / (2.0F * resolution)};
  result = doubleResInv * (data_kp1 - data_km1);
}

}  // namespace signed_distance_field
}  // namespace grid_map