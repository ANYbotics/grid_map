/*
 * test_helpers.hpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

#include <functional>
#include <vector>
#include <random>
#include "grid_map_core/TypeDefs.hpp"

namespace grid_map
{
class GridMap;
}  // namespace grid_map

namespace grid_map_test
{

/*
 * Name of the layer that is used in all tests.
 * It has no special meaning.
 */
static const char testLayer[] = "test";

/*
 * Class that holds a function pointer to analytical
 * functions used for evaluation. Analytical functions
 * are in the form of f = f(x,y). One could also add
 * derivatives to this class, e.g. for testing the
 * accuracy of the normal estimation.
 */
struct AnalyticalFunctions
{
  std::function<double(double, double)> f_;
};

struct Point2D
{
  double x_ = 0.0;
  double y_ = 0.0;
};

// Random generator engine.
extern std::mt19937 rndGenerator;

// Maximal tolerance when comparing doubles in tests.
const double maxAbsErrorValue = 1e-3;

grid_map::GridMap createMap(
  const grid_map::Length & length, double resolution,
  const grid_map::Position & pos);

/*
 * Collections of methods that modify the grid map.
 * All these methods create an analytical function that
 * describes the value of the layer "testLayer" as a function
 * of coordinates.  That can be any mathematical function. Inside the test,
 * sinusoidal, polynomial functions and exponential functions are used.
 * e.g. f(x,y) = sin(x) + cos(y), f(x,y) = exp(-x*x - y*y)
 * Grid map is then filled by evaluating
 * that analytical function over the entire length and width of the
 * grid map. Grid map thus contains spatially sampled mathematical
 * function.
 * Each method returns a structure containing the analytical function.
 */
AnalyticalFunctions createFlatWorld(grid_map::GridMap * map);
AnalyticalFunctions createRationalFunctionWorld(grid_map::GridMap * map);
AnalyticalFunctions createSaddleWorld(grid_map::GridMap * map);
AnalyticalFunctions createSecondOrderPolyWorld(grid_map::GridMap * map);
AnalyticalFunctions createSineWorld(grid_map::GridMap * map);
AnalyticalFunctions createTanhWorld(grid_map::GridMap * map);
AnalyticalFunctions createGaussianWorld(grid_map::GridMap * map);

/*
 * Iterates over the grid map and fill it with values.
 * values are calculated by evaluating analytical function.
 */
void fillGridMap(grid_map::GridMap * map, const AnalyticalFunctions & functions);

/*
 * Create numPoints uniformly distributed random points that lie within the grid map.
 */
std::vector<Point2D> uniformlyDitributedPointsWithinMap(
  const grid_map::GridMap & map,
  unsigned int numPoints);

/*
 * For each point in queryPoints, verify that the interpolated value of the grid map
 * is close to the ground truth which is contained in Analytical functions structure.
 * Called inside the tests. Calls macros from gtest.
 */
void verifyValuesAtQueryPointsAreClose(
  const grid_map::GridMap & map, const AnalyticalFunctions & trueValues,
  const std::vector<Point2D> & queryPoints,
  grid_map::InterpolationMethods interpolationMethod);

}  // namespace grid_map_test
#endif  // TEST_HELPERS_HPP_
