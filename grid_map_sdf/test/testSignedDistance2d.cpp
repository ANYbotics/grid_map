/*
 * testSignedDistance2d.cpp
 *
 *  Created on: Jul 10, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#include <gtest/gtest.h>

#include "grid_map_sdf/PixelBorderDistance.hpp"
#include "grid_map_sdf/SignedDistance2d.hpp"

#include "naiveSignedDistance.hpp"

using namespace grid_map;
using namespace signed_distance_field;

TEST(testSignedDistance2d, signedDistance2d_noObstacles) {
  const int n = 3;
  const int m = 4;
  const float resolution = 0.1;
  const Matrix map = Matrix::Ones(n, m);

  const auto occupancy = occupancyAtHeight(map, 2.0);
  const auto signedDistance = signedDistanceFromOccupancy(occupancy, resolution);

  ASSERT_TRUE((signedDistance.array() == INF).all());
}

TEST(testSignedDistance2d, signedDistance2d_allObstacles) {
  const int n = 3;
  const int m = 4;
  const float resolution = 0.1;
  const Matrix map = Matrix::Ones(n, m);

  const auto occupancy = occupancyAtHeight(map, 0.0);
  const auto signedDistance = signedDistanceFromOccupancy(occupancy, resolution);

  ASSERT_TRUE((signedDistance.array() == -INF).all());
}

TEST(testSignedDistance2d, signedDistance2d_mixed) {
  const int n = 2;
  const int m = 3;
  const float resolution = 1.0;
  Matrix map(n, m);
  map << 0.0, 1.0, 1.0, 0.0, 0.0, 1.0;
  const auto occupancy = occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_oneObstacle) {
  const int n = 20;
  const int m = 30;
  const float resolution = 0.1;
  Matrix map = Matrix::Zero(n, m);
  map(n / 2, m / 2) = 1.0;
  const auto occupancy = occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_oneFreeSpace) {
  const int n = 20;
  const int m = 30;
  const float resolution = 0.1;
  Matrix map = Matrix::Ones(n, m);
  map(n / 2, m / 2) = 0.0;

  const auto occupancy = occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_debugcase) {
  const int n = 3;
  const int m = 3;
  const float resolution = 1.0;
  Matrix map(n, m);
  map << 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0;

  const auto occupancy = occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_random) {
  const int n = 20;
  const int m = 30;
  const float resolution = 1.0;
  Matrix map = Matrix::Random(n, m);  // random [-1.0, 1.0]

  // Check at different heights, resulting in different levels of sparsity.
  float heightStep = 0.1;
  for (float height = -1.0 - heightStep; height < 1.0 + heightStep; height += heightStep) {
    const auto occupancy = occupancyAtHeight(map, height);

    const auto naiveSignedDistance = naiveSignedDistanceFromOccupancy(occupancy, resolution);
    const auto signedDistance = signedDistanceFromOccupancy(occupancy, resolution);
    ASSERT_TRUE(isEqualSdf(signedDistance, naiveSignedDistance, 1e-4)) << "height: " << height;
  }
}