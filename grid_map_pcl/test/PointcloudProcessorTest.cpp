/*
 * PointcloudProcessorTest.cpp
 *
 *  Created on: Nov 21, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <cstdlib>

#include <gtest/gtest.h>

#include "grid_map_pcl/PointcloudProcessor.hpp"

#include "PointcloudCreator.hpp"
#include "test_helpers.hpp"

namespace grid_map {
namespace grid_map_pcl_test {

TEST(PointcloudProcessorTest, ExtractClusters) {
  grid_map_pcl_test::setVerbosityLevel(ros::console::levels::Warn);
  const auto seed = rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const unsigned int numTests = 10;
  for (unsigned int i = 0; i < numTests; ++i) {
    double minZ, stdDevZ;
    int nBlobs;
    auto cloud = grid_map_pcl_test::PointcloudCreator::createNBlobsAboveEachOther(&minZ, &stdDevZ, &nBlobs);
    grid_map::grid_map_pcl::PointcloudProcessor pointcloudProcessor;
    pointcloudProcessor.loadParameters(grid_map_pcl_test::getConfigFilePath());
    auto clusters = pointcloudProcessor.extractClusterCloudsFromPointcloud(cloud);
    EXPECT_EQ(clusters.size(), nBlobs);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test ExtractClusters failed with seed: " << seed << std::endl;
  }
}

} /*namespace grid_map_pcl_test */
}  // namespace grid_map
