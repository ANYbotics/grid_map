/*
 * CubicInterpolationTest.cpp
 *
 *  Created on: Mar 12, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

// gtest
#include <gtest/gtest.h>

#include "test_helpers.hpp"

#include "grid_map_core/GridMap.hpp"

namespace gm = grid_map;
namespace gmt = grid_map_test;

TEST(CubicInterpolation, FlatWorld)
{
  const int seed = std::rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(2.0, 2.0), 0.1, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createFlatWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 100);

  verifyValuesAtQueryPointsAreClose(
    map, trueValues, queryPoints,
    gm::InterpolationMethods::INTER_CUBIC);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicInterpolation, FlatWorld failed with seed: " << seed << std::endl;
  }
}

TEST(CubicInterpolation, RationalFunctionWorld)
{
  const int seed = std::rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.01, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createRationalFunctionWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(
    map, trueValues, queryPoints,
    gm::InterpolationMethods::INTER_CUBIC);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicInterpolation, RationalFunctionWorld failed with seed: " << seed <<
      std::endl;
  }
}

TEST(CubicInterpolation, SaddleWorld)
{
  const int seed = std::rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.1, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createSaddleWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(
    map, trueValues, queryPoints,
    gm::InterpolationMethods::INTER_CUBIC);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicInterpolation, SaddleWorld failed with seed: " << seed << std::endl;
  }
}

TEST(CubicInterpolation, SecondOrderPolyWorld)
{
  const int seed = std::rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.1, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createSecondOrderPolyWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(
    map, trueValues, queryPoints,
    gm::InterpolationMethods::INTER_CUBIC);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicInterpolation, SecondOrderPolyWorld failed with seed: " << seed <<
      std::endl;
  }
}

TEST(CubicInterpolation, SineWorld)
{
  const int seed = std::rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.01, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createSineWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(
    map, trueValues, queryPoints,
    gm::InterpolationMethods::INTER_CUBIC);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicInterpolation, SineWorld failed with seed: " << seed << std::endl;
  }
}

TEST(CubicInterpolation, TanhWorld)
{
  const int seed = std::rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(2.5, 2.5), 0.02, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createTanhWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(
    map, trueValues, queryPoints,
    gm::InterpolationMethods::INTER_CUBIC);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicInterpolation, TanhWorld failed with seed: " << seed << std::endl;
  }
}

TEST(CubicInterpolation, GaussianWorld)
{
  const int seed = std::rand();
  gmt::rndGenerator.seed(seed);
  auto map = gmt::createMap(gm::Length(3.0, 3.0), 0.02, gm::Position(0.0, 0.0));
  auto trueValues = gmt::createGaussianWorld(&map);
  const auto queryPoints = gmt::uniformlyDitributedPointsWithinMap(map, 1000);

  verifyValuesAtQueryPointsAreClose(
    map, trueValues, queryPoints,
    gm::InterpolationMethods::INTER_CUBIC);

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CubicInterpolation, GaussianWorld failed with seed: " << seed <<
      std::endl;
  }
}
