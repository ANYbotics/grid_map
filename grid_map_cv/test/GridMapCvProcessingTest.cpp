/*
 * GridMapCvProcessingTest.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Peter Fankhauser
 */

#include "grid_map_cv/grid_map_cv.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/gtest_eigen.hpp>

// gtest
#include <gtest/gtest.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace grid_map;

TEST(GridMapCvProcessing, changeResolution) {
  // Create grid map.
  GridMap mapIn;
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn.add("layer", 1.0);
  for (grid_map::CircleIterator iterator(mapIn, mapIn.getPosition(), 0.2); !iterator.isPastEnd(); ++iterator) {
    mapIn.at("layer", *iterator) = 2.0;
  }

  // Change resolution.
  GridMap mapOut;
  EXPECT_TRUE(GridMapCvProcessing::changeResolution(mapIn, mapOut, 0.1));

  // Check data.
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE(mapIn.getPosition() == mapOut.getPosition());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize() * 10).all());
  EXPECT_EQ(mapIn["layer"](0, 0), mapOut["layer"](0, 0));                                                       // Corner.
  EXPECT_EQ(mapIn.atPosition("layer", mapIn.getPosition()), mapOut.atPosition("layer", mapOut.getPosition()));  // Center.
}

TEST(GridMapCvProcessing, changeResolutionForMovedMap) {
  // Create grid map.
  GridMap mapIn;
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  Position position(0.3, 0.4);
  mapIn.move(position);
  mapIn.add("layer", 1.0);
  for (grid_map::CircleIterator iterator(mapIn, position, 0.2); !iterator.isPastEnd(); ++iterator) {
    mapIn.at("layer", *iterator) = 2.0;
  }

  // Change resolution.
  GridMap mapOut;
  EXPECT_TRUE(GridMapCvProcessing::changeResolution(mapIn, mapOut, 0.1));

  // Check data.
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE(mapIn.getPosition() == mapOut.getPosition());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize() * 10).all());
  EXPECT_EQ(mapIn["layer"](0, 0), mapOut["layer"](0, 0));                                                       // Corner.
  EXPECT_EQ(mapIn.atPosition("layer", mapIn.getPosition()), mapOut.atPosition("layer", mapOut.getPosition()));  // Center.
}

/**
 * We check that the marker we set in the top left corner is rotated accordingly.
 *
 *         Rotate by 90° counter-clockwise.
 *      +------+
 *      |1     |
 *      |      |          +------------+
 *      |      |          |            |
 *      |      |  +--->   |1           |
 *      |      |          +------------+
 *      |      |
 *      +------+
 */
TEST(GridMap, TransformRotate90) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";

  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());
  map.get(heightLayerName)(0, 0) = 1.0;

  // Transformation (90° rotation).
  Eigen::Isometry3d transform = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(M_PI / 2, Vector3::UnitZ());

  // Apply affine transformation.
  const GridMap transformedMap = GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId());

  // Check if map has been rotated by 90° about z
  // Check dimensions.
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().y(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().x(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());

  // Check if marker was rotated.
  EXPECT_DOUBLE_EQ(map.get(heightLayerName)(0, 0), transformedMap.get(heightLayerName)(19, 0));
  EXPECT_DOUBLE_EQ(transformedMap.get(heightLayerName)(19, 0), 1.0);

  // Check map position.
  EXPECT_DOUBLE_EQ(map.getPosition().x(), transformedMap.getPosition().x());
  EXPECT_DOUBLE_EQ(map.getPosition().y(), transformedMap.getPosition().y());
}

/**
 * We check that the marker we set in the top left corner is rotated accordingly.
 *
 *         Rotate by 180° counter-clockwise.
 *         +------+         +------+
 *         |1     |         |      |
 *         |      |         |      |
 *         |      |         |      |
 *         |      |  +--->  |      |
 *         |      |         |      |
 *         |      |         |     1|
 *         +------+         +------+
 */
TEST(GridMap, TransformRotate180) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";

  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());
  map.get(heightLayerName)(0, 0) = 1.0;

  // Transformation (180° rotation).
  Eigen::Isometry3d transform = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(M_PI, Vector3::UnitZ());

  // Apply affine transformation.
  const GridMap transformedMap = GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId());

  // Check if map has been rotated by 180° about z

  // Check dimensions.
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().x(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().y(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());

  // Check if marker was rotated.
  EXPECT_DOUBLE_EQ(map.get(heightLayerName)(0, 0), transformedMap.get(heightLayerName)(9, 19));
  EXPECT_DOUBLE_EQ(transformedMap.get(heightLayerName)(9, 19), 1.0);

  // Check map position.
  EXPECT_DOUBLE_EQ(map.getPosition().x(), transformedMap.getPosition().x());
  EXPECT_DOUBLE_EQ(map.getPosition().y(), transformedMap.getPosition().y());
}

/**
 * We check that the marker we set in the top left corner is rotated accordingly.
 *
 *         Rotate by 270° counter-clockwise / 90° clockwise.
 *      +------+
 *      |1     |
 *      |      |          +------------+
 *      |      |          |           1|
 *      |      |  +--->   |            |
 *      |      |          +------------+
 *      |      |
 *      +------+
 */
TEST(GridMap, TransformRotate270) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";

  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());
  map.get(heightLayerName)(0, 0) = 1.0;

  // Transformation (270° rotation).
  Eigen::Isometry3d transform = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(-M_PI / 2, Vector3::UnitZ());

  // Apply affine transformation.
  const GridMap transformedMap = GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId());

  // Check if map has been rotated by 270° about z

  // Check dimensions.
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().y(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().x(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());

  // Check if marker was rotated.
  EXPECT_DOUBLE_EQ(map.get(heightLayerName)(0, 0), transformedMap.get(heightLayerName)(0, 9));
  EXPECT_DOUBLE_EQ(transformedMap.get(heightLayerName)(0, 9), 1.0);

  // Check map position.
  EXPECT_DOUBLE_EQ(map.getPosition().x(), transformedMap.getPosition().x());
  EXPECT_DOUBLE_EQ(map.getPosition().y(), transformedMap.getPosition().y());
}


/**
 * We check that the marker we set in the top left corner is rotated accordingly.
 *
 *         Rotate by 36° counter-clockwise.
 *         +------+         +------+
 *         |1     |         |1     |
 *         |      |         |      |
 *         |      |         |      |
 *         |      |  +--->  |      |
 *         |      |         |      |
 *         |      |         |      |
 *         +------+         +------+
 */
TEST(GridMap, TransformRotate360) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";

  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());
  map.get(heightLayerName)(0, 0) = 1.0;

  // Transformation (360° rotation).
  Eigen::Isometry3d transform = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(M_PI * 2, Vector3::UnitZ());

  // Apply affine transformation.
  const GridMap transformedMap = GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId());

  // Check if map has been rotated by 360° about z

  // Check dimensions
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().x(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().y(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());

  // Check if marker was rotated.
  EXPECT_DOUBLE_EQ(map.get(heightLayerName)(0, 0), transformedMap.get(heightLayerName)(0, 0));
  EXPECT_DOUBLE_EQ(transformedMap.get(heightLayerName)(0, 0), 1.0);

  // Check map position.
  EXPECT_DOUBLE_EQ(map.getPosition().x(), transformedMap.getPosition().x());
  EXPECT_DOUBLE_EQ(map.getPosition().y(), transformedMap.getPosition().y());
}

/**
 * Expected Result:
 * ----------------
 *
 *         map   rotate by 45°    transformedMap
 *
 *
 *                            +---------------------+
 *     +--------+             |     XXX             |
 *     |XXXXXXXX|             |   XXXXXXX           |
 *     |XXXXXXXX|             | XXXXXXXXXXXX        |
 *     |XXXXXXXX|             | XXXXXXXXXXXXXXX     |
 *     |XXXXXXXX|             |XXXXXXXXXXXXXXXXX    |
 * 2m  |XXXXXXXX|   +----->   | XXXXXXXXXXXXXXXXXX  |  sin(45°) * 1m + cos(45°) * 2m
 *     |XXXXXXXX|             |   XXXXXXXXXXXXXXXXXX|
 *     |XXXXXXXX|             |      XXXXXXXXXXXXXXX|  = 2.121m
 *     |XXXXXXXX|             |        XXXXXXXXXXXXX|
 *     |XXXXXXXX|             |          XXXXXXXX   |
 *     +--------+             |             XXX     |
 *         1m                 +---------------------+
 *                                     2.121m
 */
TEST(GridMap, TransformRotate45) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";
  double resolution = 0.02;
  map.setGeometry(Length(1.0, 2.0), resolution, Position(0.0, 0.0));
  map.add(heightLayerName, 1.0);
  map.setBasicLayers(map.getLayers());

  // Transformation (45° rotation).
  Eigen::Isometry3d transform = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(M_PI / 4, Vector3::UnitZ());

  // Apply affine transformation.
  const GridMap transformedMap = GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId());

  // Check if map has been rotated by 45° about z

  // Check dimensions.
  EXPECT_NEAR(transformedMap.getLength().x(), 2.121, resolution);
  EXPECT_NEAR(transformedMap.getLength().y(), 2.121, resolution);

  // Check that filled area stays the same.
  EXPECT_TRUE(transformedMap.get(heightLayerName).hasNaN());
  EXPECT_NEAR(transformedMap.get(heightLayerName).sumOfFinites() / map.get(heightLayerName).size(), 1.0, 1e-2);
}

TEST(GridMap, TransformTranslationXZY) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";
  const auto otherLayerName = "other_layer";
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.add(otherLayerName, 0.0);
  map.setBasicLayers(map.getLayers());

  // Translation by (1, -0.5, 0.2):
  Eigen::Isometry3d transform = Eigen::Translation3d(1, -0.5, 0.2) * Eigen::AngleAxisd(0, Vector3::UnitZ());

  // Apply affine transformation.
  const GridMap transformedMap = GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId());

  // Check if map has still the same size
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().x(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().y(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());

  // Check if position was updated.
  EXPECT_NEAR(transformedMap.getPosition().x(), transform.translation().x(), 1e-6);
  EXPECT_NEAR(transformedMap.getPosition().y(), transform.translation().y(), 1e-6);

  // Check if height values were updated.
  EXPECT_NEAR(map.get(heightLayerName)(0, 0) + transform.translation().z(), transformedMap.get(heightLayerName)(0, 0), 1e-6);

  // Check that other layers were kept as before.
  EXPECT_NEAR(map.get(otherLayerName)(0, 0), transformedMap.get(otherLayerName)(0, 0), 1e-6);
}

TEST(GridMap, TransformUnaligned) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());

  // A small pitch rotation by 22.5°
  Eigen::Isometry3d transform = Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(M_PI / 8, Vector3::UnitY());

  // Check that this unaligned transformation is not accepted.
  ASSERT_ANY_THROW(const GridMap transformedMap =
                       GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId()));
}

/**
 * Make sure to run this flag with optimization flags enabled. This test always succeeds.
 *
 * Local output on 01.10.2020:
 * ---------------------------
 *
 * Transforming a 2 x 2 m GridMap with a resolution of 2 cm by 45° took:
 * avg: 1.03322ms, 	min: 0.946455ms, 	max 1.51534ms
 */
TEST(GridMap, TransformComputationTime) {
  // Initial map.
  GridMap map;
  const auto heightLayerName = "height";
  map.setGeometry(Length(4.0, 4.0), 0.02, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());

  // Translation by (1, -0.5, 0.2):
  Eigen::Isometry3d transform = Eigen::Translation3d(1, -0.5, 0.2) * Eigen::AngleAxisd(M_PI / 4, Vector3::UnitZ());

  // Setup timing metrics.
  long sumTimes = 0;
  long averageTime = 0;
  long minTime = std::numeric_limits<long>::max();
  long maxTime = std::numeric_limits<long>::lowest();

  // Benchmark the function.
  size_t numberOfSamples = 100;
  for (size_t sampleNumber = 0; sampleNumber < numberOfSamples; sampleNumber++) {
    auto timestampBefore = std::chrono::high_resolution_clock::now();
    const GridMap transformedMap = GridMapCvProcessing::getTransformedMap(GridMap(map), transform, heightLayerName, map.getFrameId());
    auto timestampAfter = std::chrono::high_resolution_clock::now();

    long time = std::chrono::duration_cast<std::chrono::nanoseconds>(timestampAfter - timestampBefore).count();
    sumTimes += time;
    if (time < minTime) {
      minTime = time;
    }
    if (time > maxTime) {
      maxTime = time;
    }
  }
  averageTime = sumTimes / numberOfSamples;

  std::cout << "Transforming a 2 x 2 m GridMap with a resolution of 2 cm by 45° took: " << std::endl
            << "avg: " << averageTime * 1e-6f << "ms, \tmin: " << minTime * 1e-6f << "ms, \tmax " << maxTime * 1e-6f << "ms." << std::endl;
}
