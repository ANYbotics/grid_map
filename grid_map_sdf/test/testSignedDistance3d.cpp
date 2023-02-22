/*
 * testSignedDistance3d.cpp
 *
 *  Created on: Aug 10, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#include <gtest/gtest.h>

#include "grid_map_sdf/PixelBorderDistance.hpp"
#include "grid_map_sdf/SignedDistance2d.hpp"
#include "grid_map_sdf/SignedDistanceField.hpp"

#include "naiveSignedDistance.hpp"

using namespace grid_map;
using namespace signed_distance_field;

TEST(testSignedDistance3d, flatTerrain) {
  const int n = 3;
  const int m = 4;
  const float resolution = 0.1;
  const float terrainHeight = 0.5;
  const Matrix map = Matrix::Constant(n, m, terrainHeight);
  const float minHeight = map.minCoeff();
  const float maxHeight = map.maxCoeff();

  const float testHeightAboveTerrain = 3.0;
  const auto naiveSignedDistanceAbove = naiveSignedDistanceAtHeight(map, testHeightAboveTerrain, resolution);
  const auto signedDistanceAbove = signedDistanceAtHeight(map, testHeightAboveTerrain, resolution, minHeight, maxHeight);
  ASSERT_TRUE(isEqualSdf(signedDistanceAbove, naiveSignedDistanceAbove, 1e-4));

  const float testHeightBelowTerrain = -3.0;
  const auto naiveSignedDistanceBelow = naiveSignedDistanceAtHeight(map, testHeightBelowTerrain, resolution);
  const auto signedDistanceBelow = signedDistanceAtHeight(map, testHeightBelowTerrain, resolution, minHeight, maxHeight);
  ASSERT_TRUE(isEqualSdf(signedDistanceBelow, naiveSignedDistanceBelow, 1e-4));
}

TEST(testSignedDistance3d, randomTerrain) {
  const int n = 20;
  const int m = 30;
  const float resolution = 0.1;
  Matrix map = Matrix::Random(n, m);  // random [-1.0, 1.0]
  const float minHeight = map.minCoeff();
  const float maxHeight = map.maxCoeff();

  // Check at different heights, resulting in different levels of sparsity.
  float heightStep = 0.1;
  for (float height = -1.0 - heightStep; height < 1.0 + heightStep; height += heightStep) {
    const auto naiveSignedDistance = naiveSignedDistanceAtHeight(map, height, resolution);
    const auto signedDistance = signedDistanceAtHeight(map, height, resolution, minHeight, maxHeight);

    ASSERT_TRUE(isEqualSdf(signedDistance, naiveSignedDistance, 1e-4)) << "height: " << height;
  }
}

TEST(testSignedDistance3d, randomTerrainInterpolation) {
  const int n = 20;
  const int m = 30;
  const float resolution = 0.1;
  GridMap map;
  map.setGeometry({n * resolution, m * resolution}, resolution);
  map.add("elevation");
  map.get("elevation").setRandom();  // random [-1.0, 1.0]
  const Matrix mapData = map.get("elevation");
  const float minHeight = mapData.minCoeff();
  const float maxHeight = mapData.maxCoeff();

  SignedDistanceField sdf(map, "elevation", minHeight, maxHeight);

  // Check at different heights/
  for (float height = minHeight; height < maxHeight; height += resolution) {
    const auto naiveSignedDistance = naiveSignedDistanceAtHeight(mapData, height, resolution);

    for (int i = 0; i < mapData.rows(); ++i) {
      for (int j = 0; j < mapData.rows(); ++j) {
        Position position2d;
        map.getPosition({i, j}, position2d);

        const auto sdfValue = sdf.value({position2d.x(), position2d.y(), height});
        const auto sdfCheck = naiveSignedDistance(i, j);
        ASSERT_LT(std::abs(sdfValue - sdfCheck), 1e-4);
      }
    }
  }
}

TEST(testSignedDistance3d, randomTerrainDerivative) {
  const int n = 10;
  const int m = 20;
  const float resolution = 0.1;
  GridMap map;
  map.setGeometry({n * resolution, m * resolution}, resolution);
  map.add("elevation");
  map.get("elevation").setRandom();  // random [-1.0, 1.0]
  const Matrix mapData = map.get("elevation");
  const float minHeight = mapData.minCoeff();
  const float maxHeight = mapData.maxCoeff();

  SignedDistanceField sdf(map, "elevation", minHeight, maxHeight);

  // Check at different heights/
  int numLayers = (maxHeight - minHeight) / resolution;
  for (int k = 0; k <= numLayers; ++k) {
    const float height = minHeight + k * resolution;
    const auto naiveSignedDistance = naiveSignedDistanceAtHeight(mapData, height, resolution);
    const auto naiveSignedDistanceNext = naiveSignedDistanceAtHeight(mapData, height + resolution, resolution);
    const auto naiveSignedDistancePrevious = naiveSignedDistanceAtHeight(mapData, height - resolution, resolution);

    for (int i = 0; i < mapData.rows(); ++i) {
      for (int j = 0; j < mapData.rows(); ++j) {
        Position position2d;
        map.getPosition({i, j}, position2d);
        const auto sdfderivative = sdf.valueAndDerivative(Position3{position2d.x(), position2d.y(), height});
        const auto sdfCheck = naiveSignedDistance(i, j);
        ASSERT_LT(std::abs(sdfderivative.first - sdfCheck), 1e-4);

        // Check finite difference
        float dx = 0.0;
        if (i > 0) {
          if (i + 1 < mapData.rows()) {
            dx = (naiveSignedDistance(i + 1, j) - naiveSignedDistance(i - 1, j)) / (-2.0 * resolution);
          } else {
            dx = (naiveSignedDistance(i, j) - naiveSignedDistance(i - 1, j)) / (-resolution);
          }
        } else {
          dx = (naiveSignedDistance(i + 1, j) - naiveSignedDistance(i, j)) / (-resolution);
        }
        ASSERT_LT(std::abs(dx - sdfderivative.second.x()), 1e-4);

        float dy = 0.0;
        if (j > 0) {
          if (j + 1 < mapData.cols()) {
            dy = (naiveSignedDistance(i, j + 1) - naiveSignedDistance(i, j - 1)) / (-2.0 * resolution);
          } else {
            dy = (naiveSignedDistance(i, j) - naiveSignedDistance(i, j - 1)) / (-resolution);
          }
        } else {
          dy = (naiveSignedDistance(i, j + 1) - naiveSignedDistance(i, j)) / (-resolution);
        }
        ASSERT_LT(std::abs(dy - sdfderivative.second.y()), 1e-4);

        float dz = 0.0;
        if (k > 0) {
          if (k < numLayers) {
            dz = (naiveSignedDistanceNext(i, j) - naiveSignedDistancePrevious(i, j)) / (2.0 * resolution);
          } else {
            dz = (naiveSignedDistance(i, j) - naiveSignedDistancePrevious(i, j)) / (resolution);
          }
        } else {
          dz = (naiveSignedDistanceNext(i, j) - naiveSignedDistance(i, j)) / (resolution);
        }
        ASSERT_LT(std::abs(dz - sdfderivative.second.z()), 1e-4);
      }
    }
  }
}

TEST(testSignedDistance3d, extrapolation) {
  const int n = 20;
  const int m = 30;
  const float resolution = 0.1;
  const float h = 0.5;
  GridMap map;
  map.setGeometry({n * resolution, m * resolution}, resolution);
  map.add("elevation");
  map.get("elevation").setConstant(h);  // random [-1.0, 1.0]
  const Matrix mapData = map.get("elevation");
  const float minHeight = h - resolution;
  const float maxHeight = h + resolution;

  SignedDistanceField sdf(map, "elevation", minHeight, maxHeight);

  // Check at different heights/
  for (float height = h - 1.0; height < h + 1.0; height += resolution) {
    const auto naiveSignedDistance = naiveSignedDistanceAtHeight(mapData, height, resolution);

    for (int i = 0; i < mapData.rows(); ++i) {
      for (int j = 0; j < mapData.rows(); ++j) {
        Position position2d;
        map.getPosition({i, j}, position2d);

        const auto sdfValueAndDerivative = sdf.valueAndDerivative({position2d.x(), position2d.y(), height});
        const auto sdfCheck = naiveSignedDistance(i, j);
        ASSERT_LT(std::abs(sdfValueAndDerivative.first - sdfCheck), 1e-4);

        // Constant terrain, derivative should be up everywhere
        ASSERT_LT((sdfValueAndDerivative.second - SignedDistanceField::Derivative3::UnitZ()).norm(), 1e-4);
      }
    }
  }
}