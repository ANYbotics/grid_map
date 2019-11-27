/*
 * HelpersTest.cpp
 *
 *  Created on: Nov 21, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <cstdlib>

#include <gtest/gtest.h>

#include "grid_map_pcl/helpers.hpp"

#include "PointcloudCreator.hpp"
#include "test_helpers.hpp"


namespace grid_map {
namespace grid_map_pcl_test {

TEST(HelpersTest, MeanPositionTest)
{
  grid_map_pcl_test::setVerbosityLevel(ros::console::levels::Warn);
  const auto seed = rand();
  grid_map_pcl_test::rndGenerator.seed(seed);

  const unsigned int numTests = 10;
  for (unsigned int i = 0; i < numTests; ++i) {

    double mean, stdDev;
    auto cloud = grid_map_pcl_test::PointcloudCreator::createBlobOfPoints(&mean, &stdDev);
    auto meanOfAllPoints = grid_map_pcl::calculateMeanOfPointPositions(cloud);

    EXPECT_NEAR(meanOfAllPoints.x(), mean, 3 * stdDev);
    EXPECT_NEAR(meanOfAllPoints.y(), mean, 3 * stdDev);
    EXPECT_NEAR(meanOfAllPoints.z(), mean, 3 * stdDev);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test MeanPositionTest failed with seed: " << seed << std::endl;
  }

}

} /*namespace grid_map_pcl_test */
} /*namesapce grid_map */

