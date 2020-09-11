/*
 * test_helpers.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "test_helpers.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

// gtest
#include <gtest/gtest.h>

namespace grid_map_test {

std::mt19937 rndGenerator;

AnalyticalFunctions createFlatWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  func.f_ = [](double x, double y) {
    return 0.0;
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createRationalFunctionWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> shift(-3.0, 3.0);
  std::uniform_real_distribution<double> scale(1.0, 20.0);
  const double x0 = shift(rndGenerator);
  const double y0 = shift(rndGenerator);
  const double s = scale(rndGenerator);

  func.f_ = [x0, y0,s](double x, double y) {
    return s / (1 + std::pow(x-x0, 2.0) + std::pow(y-y0, 2.0));
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createSecondOrderPolyWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  func.f_ = [](double x,double y) {
    return (-x*x -y*y +2.0*x*y +x*x*y*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createSaddleWorld(grid_map::GridMap *map)
{
  AnalyticalFunctions func;

  func.f_ = [](double x,double y) {
    return (x*x-y*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createSineWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> Uw(0.1, 4.0);
  const double w1 = Uw(rndGenerator);
  const double w2 = Uw(rndGenerator);
  const double w3 = Uw(rndGenerator);
  const double w4 = Uw(rndGenerator);

  func.f_ = [w1,w2,w3,w4](double x,double y) {
    return std::cos(w1*x) + std::sin(w2*y) + std::cos(w3*x) + std::sin(w4*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createTanhWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> scaling(0.1, 2.0);
  const double s = scaling(rndGenerator);
  func.f_ = [s](double x,double y) {
    const double expZ = std::exp(2 *s* x);
    return (expZ - 1) / (expZ + 1);
  };

  fillGridMap(map, func);

  return func;
}

AnalyticalFunctions createGaussianWorld(grid_map::GridMap *map)
{

  struct Gaussian
  {
    double x0, y0;
    double varX, varY;
    double s;
  };

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> var(0.1, 3.0);
  std::uniform_real_distribution<double> mean(-4.0, 4.0);
  std::uniform_real_distribution<double> scale(-3.0, 3.0);
  constexpr int numGaussians = 3;
  std::array<Gaussian, numGaussians> g;

  for (int i = 0; i < numGaussians; ++i) {
    g.at(i).x0 = mean(rndGenerator);
    g.at(i).y0 = mean(rndGenerator);
    g.at(i).varX = var(rndGenerator);
    g.at(i).varY = var(rndGenerator);
    g.at(i).s = scale(rndGenerator);
  }

  func.f_ = [g](double x,double y) {
    double value = 0.0;
    for (int i = 0; i < g.size(); ++i) {
      const double x0 = g.at(i).x0;
      const double y0 = g.at(i).y0;
      const double varX = g.at(i).varX;
      const double varY = g.at(i).varY;
      const double s = g.at(i).s;
        value += s * std::exp(-(x-x0)*(x-x0) / (2.0*varX) - (y-y0)*(y-y0) / (2.0 * varY));
    }

    return value;
  };

  fillGridMap(map, func);

  return func;
}

void fillGridMap(grid_map::GridMap *map, const AnalyticalFunctions &functions)
{
  grid_map::Matrix& data = (*map)[testLayer];
  for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position pos;
    map->getPosition(index, pos);
    data(index(0), index(1)) = functions.f_(pos.x(), pos.y());
  }
}

grid_map::GridMap createMap(const grid_map::Length &length, double resolution,
                            const grid_map::Position &pos)
{
  grid_map::GridMap map;

  map.setGeometry(length, resolution, pos);
  map.add(testLayer, 0.0);
  map.setFrameId("map");

  return map;
}

std::vector<Point2D> uniformlyDitributedPointsWithinMap(const grid_map::GridMap &map,
                                                       unsigned int numPoints)
{

  // stay away from the edges
  // on the edges the cubic interp is invalid. Not enough points.
  const double dimX = map.getLength().x() / 2.0 - 3.0 * map.getResolution();
  const double dimY = map.getLength().y() / 2.0 - 3.0 * map.getResolution();
  std::uniform_real_distribution<double> Ux(-dimX, dimX);
  std::uniform_real_distribution<double> Uy(-dimY, dimY);

  std::vector<Point2D> points(numPoints);
  for (auto &point : points) {
    point.x_ = Ux(rndGenerator);
    point.y_ = Uy(rndGenerator);
  }

  return points;
}

void verifyValuesAtQueryPointsAreClose(const grid_map::GridMap &map, const AnalyticalFunctions &trueValues,
                               const std::vector<Point2D> &queryPoints,
                               grid_map::InterpolationMethods interpolationMethod){
  for (const auto point : queryPoints) {
    const grid_map::Position p(point.x_, point.y_);
    const double trueValue = trueValues.f_(p.x(), p.y());
    const double interpolatedValue = map.atPosition(
        grid_map_test::testLayer, p, interpolationMethod);
    EXPECT_NEAR(trueValue, interpolatedValue, grid_map_test::maxAbsErrorValue);
  }
}


} /* namespace grid_map_test */

