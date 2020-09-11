/*
 * InterpolationDemo.cpp
 *
 *  Created on: Mar 16, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_demos/InterpolationDemo.hpp"

#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_msgs/GridMap.h"
#include "grid_map_ros/GridMapRosConverter.hpp"

namespace grid_map_demos {

AnalyticalFunctions createWorld(Worlds world, double highResolution, double lowResolution,
                                double length, double width, grid_map::GridMap *groundTruthHighRes,
                                grid_map::GridMap *groundTruthLowRes)
{

  const grid_map::Length mapLength(length, width);
  const grid_map::Position mapPosition(0.0, 0.0);
  *groundTruthHighRes = createMap(mapLength, highResolution, mapPosition);
  *groundTruthLowRes = createMap(mapLength, lowResolution, mapPosition);
  AnalyticalFunctions groundTruth;
  switch (world) {

    case Worlds::Sine: {
      groundTruth = createSineWorld(groundTruthHighRes);
      createSineWorld(groundTruthLowRes);
      break;
    }
    case Worlds::Tanh: {
      groundTruth = createTanhWorld(groundTruthHighRes);
      createTanhWorld(groundTruthLowRes);
      break;
    }
    case Worlds::GaussMixture: {
      groundTruth = createGaussianWorld(groundTruthHighRes);
      createGaussianWorld(groundTruthLowRes);
      break;
    }
    case Worlds::Poly: {
      groundTruth = createPolyWorld(groundTruthHighRes);
      createPolyWorld(groundTruthLowRes);
      break;
    }

    default:
      throw std::runtime_error("Interpolation demo: Unknown world requested.");
  }

  return groundTruth;
}

AnalyticalFunctions createPolyWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  func.f_ = [](double x,double y) {
    return (-x*x -y*y +2.0*x*y +x*x*y*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createSineWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  const double w1 = 1.0;
  const double w2 = 2.0;
  const double w3 = 3.0;
  const double w4 = 4.0;

  func.f_ = [w1,w2,w3,w4](double x,double y) {
    return std::cos(w1*x) + std::sin(w2*y) + std::cos(w3*x) + std::sin(w4*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createTanhWorld(grid_map::GridMap *map)
{

  AnalyticalFunctions func;

  const double s = 5.0;
  func.f_ = [s](double x,double y) {
    const double expZx = std::exp(2 *s* x);
    const double tanX = (expZx - 1) / (expZx + 1);
    const double expZy = std::exp(2 *s* y);
    const double tanY = (expZy - 1) / (expZy + 1);
    return tanX + tanY;
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
  constexpr int numGaussians = 7;
  std::array<std::pair<double, double>, numGaussians> vars = { { { 1.0, 0.3 }, { 0.25, 0.25 }, {
      0.1, 0.1 }, { 0.1, 0.1 }, { 0.1, 0.1 }, { 0.1, 0.05 }, { 0.05, 0.05 } } };
  std::array<std::pair<double, double>, numGaussians> means = { { { 1, -1 }, { 1, 1.7 },
      { -1, 1.6 }, { -1.8, -1.8 }, { -1, 1.8 }, { 0, 0 }, { -1.2, 0 } } };
  std::array<double, numGaussians> scales = { -2.0, -1.0, 2.0, 1.0, 3.0, 4.0, 1.0 };

  std::array<Gaussian, numGaussians> g;

  for (int i = 0; i < numGaussians; ++i) {
    g.at(i).x0 = means.at(i).first;
    g.at(i).y0 = means.at(i).second;
    g.at(i).varX = vars.at(i).first;
    g.at(i).varY = vars.at(i).second;
    g.at(i).s = scales.at(i);
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
  grid_map::Matrix& data = (*map)[demoLayer];
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
  map.add(demoLayer, 0.0);
  map.setFrameId("map");

  return map;
}

grid_map::GridMap createInterpolatedMapFromDataMap(const grid_map::GridMap &dataMap,
                                                   double desiredResolution)
{
  grid_map::GridMap interpolatedMap;
  interpolatedMap.setGeometry(dataMap.getLength(), desiredResolution, dataMap.getPosition());
  const std::string &layer = demoLayer;
  interpolatedMap.add(layer, 0.0);
  interpolatedMap.setFrameId(dataMap.getFrameId());

  return interpolatedMap;
}

void interpolateInputMap(const grid_map::GridMap &dataMap,
                         grid_map::InterpolationMethods interpolationMethod,
                         grid_map::GridMap *interpolatedMap)
{
  for (grid_map::GridMapIterator iterator(*interpolatedMap); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position pos;
    interpolatedMap->getPosition(index, pos);
    const double interpolatedHeight = dataMap.atPosition(demoLayer, pos, interpolationMethod);
    interpolatedMap->at(demoLayer, index) = interpolatedHeight;
  }
}

Error computeInterpolationError(const AnalyticalFunctions &groundTruth,
                                const grid_map::GridMap &map)
{
  const double r = map.getResolution();
  const double dimX = map.getLength().x() / 2.0 - 3.0 * r;
  const double dimY = map.getLength().y() / 2.0 - 3.0 * r;

  unsigned int count = 0;
  Error error;
  const int nRow = map.getSize().x();
  const int nCol = map.getSize().y();
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const auto row = (*iterator).x();
    const auto col = (*iterator).y();
    const bool skipEvaluation = row < 2 || col < 2 || col > (nCol - 3) || row > (nRow - 3);
    if (skipEvaluation) {
      continue;
    }
    grid_map::Position pos;
    map.getPosition(*iterator, pos);
    const double f = map.at(demoLayer, *iterator);
    const double f_ = groundTruth.f_(pos.x(), pos.y());
    const double e = std::fabs(f - f_);
    error.meanSquare_ += e * e;
    error.meanAbs_ += e;
    ++count;
    if (e > error.max_) {
      error.max_ = e;
    }
  }
  error.meanSquare_ /= count;
  error.meanAbs_ /= count;
  return error;

}

InterpolationDemo::InterpolationDemo(ros::NodeHandle *nh)
{

  nh->param<std::string>("interpolation_type", interpolationMethod_, "Nearest");
  nh->param<std::string>("world", world_, "Sine");
  nh->param<double>("groundtruth_resolution", groundTruthResolution_, 0.02);
  nh->param<double>("interpolation/data_resolution", dataResolution_, 0.1);
  nh->param<double>("interpolation/interpolated_resolution", interpolatedResolution_, 0.02);
  nh->param<double>("world_size/length", worldLength_, 4.0);
  nh->param<double>("world_size/width", worldWidth_, 4.0);

  groundTruthMapPub_ = nh->advertise<grid_map_msgs::GridMap>("ground_truth", 1, true);
  dataSparseMapPub_ = nh->advertise<grid_map_msgs::GridMap>("data_sparse", 1, true);
  interpolatedMapPub_ = nh->advertise<grid_map_msgs::GridMap>("interpolated", 1, true);

  runDemo();
}

void InterpolationDemo::runDemo()
{

  // visualize stuff
  groundTruth_ = createWorld(worlds.at(world_), groundTruthResolution_, dataResolution_,
                             worldLength_, worldWidth_, &groundTruthMap_, &dataSparseMap_);

  interpolatedMap_ = createInterpolatedMapFromDataMap(dataSparseMap_, interpolatedResolution_);
  interpolateInputMap(dataSparseMap_, interpolationMethods.at(interpolationMethod_),
                      &interpolatedMap_);

  publishGridMaps();
  std::cout << "\n \n visualized the world: " << world_ << std::endl;

  // print some info
  const auto statistics = computeStatistics();
  printStatistics(statistics);

}

InterpolationDemo::Statistics InterpolationDemo::computeStatistics() const
{
  Statistics stats;
  for (auto world = worlds.cbegin(); world != worlds.cend(); ++world) {
    std::map<std::string, Statistic> methodsStats;
    for (auto method = interpolationMethods.cbegin(); method != interpolationMethods.cend();
        ++method) {
      const auto errorAndDuration = interpolateAndComputeError(world->first, method->first);
      Statistic statistic;
      statistic.duration_ = errorAndDuration.second;
      statistic.error_ = errorAndDuration.first;
      statistic.interpolationMethod_ = method->first;
      statistic.world_ = world->first;
      methodsStats.insert( { method->first, statistic });
    }
    stats.insert( { world->first, methodsStats });
  }

  return std::move(stats);
}

InterpolationDemo::ErrorAndDuration InterpolationDemo::interpolateAndComputeError(
    const std::string world, const std::string &method) const
{
  grid_map::GridMap highResMap, dataMap;
  auto groundTruth = createWorld(worlds.at(world), groundTruthResolution_, dataResolution_,
                                 worldLength_, worldWidth_, &highResMap, &dataMap);

  auto interpolatedMap = createInterpolatedMapFromDataMap(dataMap, interpolatedResolution_);
  const auto start = clk::now();
  interpolateInputMap(dataMap, interpolationMethods.at(method), &interpolatedMap);
  const auto end = clk::now();
  const auto count = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  const unsigned int numElements = interpolatedMap_.getSize().x() * interpolatedMap_.getSize().y();

  ErrorAndDuration errorAndDuration;
  errorAndDuration.first = computeInterpolationError(groundTruth, interpolatedMap);
  errorAndDuration.second = static_cast<double>(count) / numElements;
  return errorAndDuration;
}

void InterpolationDemo::printStatistics(const Statistics &stats) const
{
  std::cout << " \n \n ================================== \n";
  printf("Interpolated map of size: %f x %f \n", interpolatedMap_.getLength().x(),
         interpolatedMap_.getLength().y());
  printf("Resolution of the data: %f, resolution of the interpolated map: %f, \n \n",
         dataSparseMap_.getResolution(), interpolatedMap_.getResolution());

  for (auto world = worlds.cbegin(); world != worlds.cend(); ++world) {
    std::cout << "Stats for the world: " << world->first << std::endl;
    for (auto method = interpolationMethods.cbegin(); method != interpolationMethods.cend();
        ++method) {
      Statistic s = stats.at(world->first).at(method->first);
      printf(
          "Method: %-20s  Mean absolute error: %-10f     max error: %-10f  avg query duration: %-10f microsec \n",
          method->first.c_str(), s.error_.meanAbs_, s.error_.max_, s.duration_);
    }
    std::cout << std::endl;
  }
}

void InterpolationDemo::publishGridMaps() const
{
  grid_map_msgs::GridMap highResMsg, lowResMsg, interpolatedMsg;
  grid_map::GridMapRosConverter::toMessage(groundTruthMap_, highResMsg);
  grid_map::GridMapRosConverter::toMessage(dataSparseMap_, lowResMsg);
  grid_map::GridMapRosConverter::toMessage(interpolatedMap_, interpolatedMsg);
  groundTruthMapPub_.publish(highResMsg);
  dataSparseMapPub_.publish(lowResMsg);
  interpolatedMapPub_.publish(interpolatedMsg);
}

} /* namespace grid_map_demos */

