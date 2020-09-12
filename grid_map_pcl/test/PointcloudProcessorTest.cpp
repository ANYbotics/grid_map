/*
 * PointcloudProcessorTest.cpp
 *
 *  Created on: Nov 21, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>

#include <cstdlib>

#include "grid_map_pcl/PointcloudProcessor.hpp"

#include "PointcloudCreator.hpp"
#include "test_helpers.hpp"


namespace grid_map
{
namespace grid_map_pcl_test
{

TEST(PointcloudProcessorTest, ExtractClusters)
{
  rclcpp::Node node("PointcloudProcessorTest");

  auto ret = rcutils_logging_set_logger_level(
    node.get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(
      node.get_logger(),
      "Failed to change logging severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }

  const auto seed = std::rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const unsigned int numTests = 10;
  for (unsigned int i = 0; i < numTests; ++i) {
    double minZ, stdDevZ;
    int nBlobs;
    auto cloud = grid_map_pcl_test::PointcloudCreator::createNBlobsAboveEachOther(
      &minZ, &stdDevZ,
      &nBlobs);
    grid_map::grid_map_pcl::PointcloudProcessor pointcloudProcessor(node.get_logger());
    pointcloudProcessor.loadParameters(grid_map_pcl_test::getConfigFilePath());
    auto clusters = pointcloudProcessor.extractClusterCloudsFromPointcloud(cloud);
    EXPECT_EQ(clusters.size(), static_cast<uint64_t>(nBlobs));
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test ExtractClusters failed with seed: " << seed << std::endl;
  }
}

}  // namespace grid_map_pcl_test
}  // namespace grid_map
