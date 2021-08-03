/*
 * CubicConvolutionInterpolationTest.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "test_helpers.hpp"

#include "grid_map_core/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

namespace gm = grid_map;
namespace gmt = grid_map_test;

TEST(CubicConvolutionInterpolation, FlatWorld)
{
  const int seed = rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(2.0, 2.0), 0.1, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createFlatWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 100);

  verifyValuesAtQueryPointsAreClose(map, trueValues, queryPoints, gm::InterpolationMethods::INTER_CUBIC_CONVOLUTION);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicConvolutionInterpolation, FlatWorld failed with seed: " << seed
              << std::endl;
  }
}

TEST(CubicConvolutionInterpolation, RationalFunctionWorld)
{
  const int seed = rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.01, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createRationalFunctionWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(map, trueValues, queryPoints, gm::InterpolationMethods::INTER_CUBIC_CONVOLUTION);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicConvolutionInterpolation, RationalFunctionWorld failed with seed: "
              << seed << std::endl;
  }
}

TEST(CubicConvolutionInterpolation, SaddleWorld)
{
  const int seed = rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.1, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createSaddleWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(map, trueValues, queryPoints, gm::InterpolationMethods::INTER_CUBIC_CONVOLUTION);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicConvolutionInterpolation, SaddleWorld failed with seed: " << seed
              << std::endl;
  }
}

TEST(CubicConvolutionInterpolation, SecondOrderPolyWorld)
{
  const int seed = rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.1, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createSecondOrderPolyWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(map, trueValues, queryPoints, gm::InterpolationMethods::INTER_CUBIC_CONVOLUTION);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicConvolutionInterpolation, SecondOrderPolyWorld failed with seed: "
              << seed << std::endl;
  }
}

TEST(CubicConvolutionInterpolation, SineWorld)
{
  const int seed = rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.01, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createSineWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(map, trueValues, queryPoints, gm::InterpolationMethods::INTER_CUBIC_CONVOLUTION);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicConvolutionInterpolation, SineWorld failed with seed: " << seed
              << std::endl;
  }
}

TEST(CubicConvolutionInterpolation, TanhWorld)
{
  const int seed = rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(2.5, 2.5), 0.02, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createTanhWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(map, trueValues, queryPoints, gm::InterpolationMethods::INTER_CUBIC_CONVOLUTION);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicConvolutionInterpolation, TanhWorld failed with seed: " << seed
              << std::endl;
  }
}

TEST(CubicConvolutionInterpolation, GaussianWorld)
{
  const int seed = rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.02, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createGaussianWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(map, trueValues, queryPoints, gm::InterpolationMethods::INTER_CUBIC_CONVOLUTION);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicConvolutionInterpolation, GaussianWorld failed with seed: " << seed
              << std::endl;
  }
}

