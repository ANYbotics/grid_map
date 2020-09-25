/*
 * FlatGroundTest.cpp
 *
 *  Created on: Nov 4, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>

#include <cstdlib>
#include <memory>
#include <string>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "PointcloudCreator.hpp"
#include "test_helpers.hpp"

namespace grid_map
{
namespace grid_map_pcl_test
{
class GridMapPclLoaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Changing the logging verbosity to WARN
    auto ret = rcutils_logging_set_logger_level(
      test_logger_.get_name(), RCUTILS_LOG_SEVERITY_WARN);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        test_logger_, "Failed to change logging severity: %s",
        rcutils_get_error_string().str);
      rcutils_reset_error();
    }
  }

  rclcpp::Logger test_logger_ = rclcpp::get_logger("GridMapPclLoaderTest");
};

// cppcheck-suppress syntaxError
TEST_F(GridMapPclLoaderTest, FlatGroundRealDataset)
{
  grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
  gridMapPclLoader.loadParameters(grid_map_pcl_test::getConfigFilePath());
  gridMapPclLoader.loadCloudFromPcdFile(grid_map_pcl_test::getTestPcdFilePath());
  gridMapPclLoader.preProcessInputCloud();
  gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
  gridMapPclLoader.addLayerFromInputCloud(grid_map_pcl_test::layerName);

  const auto & gridMap = gridMapPclLoader.getGridMap();
  const auto elevationValues = grid_map_pcl_test::getNonNanElevationValues(gridMap);

  EXPECT_GT(elevationValues.size(), 0ul);

  // test that all the elevation values are equal
  // allow for some difference (2cm) since the input cloud is noisy (real dataset)
  double referenceElevation = elevationValues.front();
  for (const auto & elevation : elevationValues) {
    EXPECT_NEAR(elevation, referenceElevation, 2e-2);
  }
}

TEST_F(GridMapPclLoaderTest, PerfectPlane)
{
  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const int numTests = 10;
  for (int i = 0; i < numTests; ++i) {
    double height;
    auto cloud = grid_map_pcl_test::PointcloudCreator::createPerfectPlane(&height);
    grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
    grid_map_pcl_test::runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
    const auto elevationValues = grid_map_pcl_test::getNonNanElevationValues(
      gridMapPclLoader.getGridMap());

    EXPECT_GT(elevationValues.size(), 0ul);

    const double tolerance = 1e-5;
    for (const auto & elevation : elevationValues) {
      EXPECT_NEAR(elevation, height, tolerance);
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PerfectPlane failed with seed: " << seed << std::endl;
  }
}

TEST_F(GridMapPclLoaderTest, NoisyPlane)
{
  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const int numTests = 10;
  for (int i = 0; i < numTests; ++i) {
    double height, stdDevZ;
    auto cloud = grid_map_pcl_test::PointcloudCreator::createNoisyPlane(&height, &stdDevZ);
    grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
    grid_map_pcl_test::runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
    const auto elevationValues = grid_map_pcl_test::getNonNanElevationValues(
      gridMapPclLoader.getGridMap());

    EXPECT_GT(elevationValues.size(), 0ul);
    for (const auto & elevation : elevationValues) {
      EXPECT_NEAR(elevation, height, 3 * stdDevZ);
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test NoisyPlane failed with seed: " << seed << std::endl;
  }
}

TEST_F(GridMapPclLoaderTest, NoisyDoublePlane)
{
  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  double minZ, stdDevZ;
  auto cloud = grid_map_pcl_test::PointcloudCreator::createNoisyDoublePlane(&minZ, &stdDevZ);
  grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
  grid_map_pcl_test::runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
  const auto elevationValues = grid_map_pcl_test::getNonNanElevationValues(
    gridMapPclLoader.getGridMap());

  EXPECT_GT(elevationValues.size(), 0ul);
  for (const auto & elevation : elevationValues) {
    EXPECT_NEAR(elevation, minZ, 3 * stdDevZ);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test NoisyDoublePlane failed with seed: " << seed << std::endl;
  }
}

TEST_F(GridMapPclLoaderTest, InitializeGeometry)
{
  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const unsigned int numTests = 1000;
  for (unsigned int i = 0; i < numTests; ++i) {
    double xLocation, yLocation;
    auto cloud = grid_map_pcl_test::PointcloudCreator::createVerticesOfASquare(
      &xLocation,
      &yLocation);
    grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
    gridMapPclLoader.loadParameters(grid_map_pcl_test::getConfigFilePath());
    gridMapPclLoader.setInputCloud(cloud);
    gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
    auto gridMap = gridMapPclLoader.getGridMap();
    auto center = gridMap.getPosition();

    const double tolerance = 1e-5;
    EXPECT_NEAR(center.x(), 0.0, tolerance);
    EXPECT_NEAR(center.y(), 0.0, tolerance);
    auto length = gridMap.getLength();
    EXPECT_NEAR(length.x(), std::round(2 * xLocation), tolerance);
    EXPECT_NEAR(length.y(), std::round(2 * yLocation), tolerance);

    cloud->clear();
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test InitializeGeometry failed with seed: " << seed << std::endl;
  }
}

TEST_F(GridMapPclLoaderTest, NoisyStepTerrain)
{
  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const int numTests = 10;
  for (int i = 0; i < numTests; ++i) {
    double stepLocationX, zHigh, zLow, stdDevZ;
    const auto cloud = grid_map_pcl_test::PointcloudCreator::createNoisyPointcloudOfStepTerrain(
      &stepLocationX, &zHigh, &zLow, &stdDevZ);
    grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
    grid_map_pcl_test::runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
    const auto coordinates = grid_map_pcl_test::getNonNanElevationValuesWithCoordinates(
      gridMapPclLoader.getGridMap());

    EXPECT_GT(coordinates.size(), 0ul);
    for (const auto & coordinate : coordinates) {
      if (coordinate.x() > stepLocationX) {
        EXPECT_NEAR(coordinate.z(), zHigh, 3 * stdDevZ);
      }
      if (coordinate.x() < stepLocationX) {
        EXPECT_NEAR(coordinate.z(), zLow, 3 * stdDevZ);
      }
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test NoisyStepTerrain failed with seed: " << seed << std::endl;
  }
}  // end StepTerrainPointcloud test

TEST_F(GridMapPclLoaderTest, CalculateElevation)
{
  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const unsigned int numTests = 10;
  for (unsigned int i = 0; i < numTests; ++i) {
    double minZ, stdDevZ;
    int nBlobs;
    auto cloud = grid_map_pcl_test::PointcloudCreator::createNBlobsAboveEachOther(
      &minZ, &stdDevZ,
      &nBlobs);
    grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
    gridMapPclLoader.loadParameters(grid_map_pcl_test::getConfigFilePath());
    gridMapPclLoader.setInputCloud(cloud);
    double elevation = gridMapPclLoader.calculateElevationFromPointsInsideGridMapCell(cloud);
    EXPECT_NEAR(elevation, minZ, 3 * stdDevZ);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CalculateElevation failed with seed: " << seed << std::endl;
  }
}

TEST_F(GridMapPclLoaderTest, SavePointclouds)
{
  if (!grid_map_pcl_test::savePointclouds) {
    return;
  }

  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  double dummyDouble1, dummyDouble2, dummyDouble3, dummyDouble4;
  int dummyInt;

  grid_map::GridMapPclLoader gridMapPclLoader(test_logger_);
  gridMapPclLoader.loadParameters(grid_map_pcl_test::getConfigFilePath());

  // perfect plane
  std::string filename = grid_map_pcl_test::getTestDataFolderPath() + "/perfectPlane.pcd";
  auto cloud = grid_map_pcl_test::PointcloudCreator::createPerfectPlane(&dummyDouble1);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);

  // n blobs
  filename = grid_map_pcl_test::getTestDataFolderPath() + "/Nblobs.pcd";
  cloud = grid_map_pcl_test::PointcloudCreator::createNBlobsAboveEachOther(
    &dummyDouble1,
    &dummyDouble2,
    &dummyInt);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);

  // 4 points
  filename = grid_map_pcl_test::getTestDataFolderPath() + "/4pointSquare.pcd";
  cloud = grid_map_pcl_test::PointcloudCreator::createVerticesOfASquare(
    &dummyDouble1,
    &dummyDouble2);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);

  // blob
  filename = grid_map_pcl_test::getTestDataFolderPath() + "/blob.pcd";
  cloud = grid_map_pcl_test::PointcloudCreator::createBlobOfPoints(&dummyDouble1, &dummyDouble2);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);

  // step terrain
  filename = grid_map_pcl_test::getTestDataFolderPath() + "/stepTerrain.pcd";
  cloud = grid_map_pcl_test::PointcloudCreator::createNoisyPointcloudOfStepTerrain(
    &dummyDouble1,
    &dummyDouble2,
    &dummyDouble3,
    &dummyDouble4);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);

  // noisy plane
  filename = grid_map_pcl_test::getTestDataFolderPath() + "/noisyPlane.pcd";
  cloud = grid_map_pcl_test::PointcloudCreator::createNoisyPlane(&dummyDouble1, &dummyDouble2);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);

  // noisy double plane
  filename = grid_map_pcl_test::getTestDataFolderPath() + "/doublePlane.pcd";
  cloud = grid_map_pcl_test::PointcloudCreator::createNoisyDoublePlane(
    &dummyDouble1,
    &dummyDouble2);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);
}

}  // namespace grid_map_pcl_test

}  // namespace grid_map
