/*
 * grid_map_iterator_benchmark.hpp
 *
 *  Created on: Feb 15, 2016
 *     Authors: Christos Zalidis, Péter Fankhauser
 */

#include <grid_map_core/grid_map_core.hpp>
#include <chrono>
#include <iostream>
#include <string>

#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

/*!
 * Convenient use of iterator.
 */
void runGridMapIteratorVersion1(
  grid_map::GridMap & map, const std::string & layer_from,
  const std::string & layer_to)
{
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const float value_from = map.at(layer_from, *iterator);
    float & value_to = map.at(layer_to, *iterator);
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Improved efficiency by storing direct access to data layers.
 */
void runGridMapIteratorVersion2(
  grid_map::GridMap & map, const std::string & layer_from,
  const std::string & layer_to)
{
  const auto & data_from = map[layer_from];
  auto & data_to = map[layer_to];
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const float value_from = data_from(index(0), index(1));
    float & value_to = data_to(index(0), index(1));
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Improved efficiency by using linear grid_map::Index.
 */
void runGridMapIteratorVersion3(
  grid_map::GridMap & map, const std::string & layer_from,
  const std::string & layer_to)
{
  const auto & data_from = map[layer_from];
  auto & data_to = map[layer_to];
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const std::size_t i = iterator.getLinearIndex();
    const float value_from = data_from(i);
    float & value_to = data_to(i);
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Whenever possible, make use of the Eigen methods for maximum efficiency
 * and readability.
 */
void runEigenFunction(
  grid_map::GridMap & map, const std::string & layer_from,
  const std::string & layer_to)
{
  map[layer_to] = map[layer_to].cwiseMax(map[layer_from]);
}

/*!
 * For comparison.
 */
void runCustomIndexIteration(
  grid_map::GridMap & map, const std::string & layer_from,
  const std::string & layer_to)
{
  const auto & data_from = map[layer_from];
  auto & data_to = map[layer_to];
  for (int64_t j = 0; j < data_to.cols(); ++j) {
    for (int64_t i = 0; i < data_to.rows(); ++i) {
      const float value_from = data_from(i, j);
      float & value_to = data_to(i, j);
      value_to = value_to > value_from ? value_to : value_from;
    }
  }
}

/*!
 * For comparison.
 */
void runCustomLinearIndexIteration(
  grid_map::GridMap & map, const std::string & layer_from,
  const std::string & layer_to)
{
  const auto & data_from = map[layer_from];
  auto & data_to = map[layer_to];
  for (int64_t i = 0; i < data_to.size(); ++i) {
    data_to(i) = data_to(i) > data_from(i) ? data_to(i) : data_from(i);
  }
}

int main()
{
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(20.0, 20.0), 0.004, grid_map::Position(0.0, 0.0));
  map.add("random");
  map["random"].setRandom();
  map.add("layer1", 0.0);
  map.add("layer2", 0.0);
  map.add("layer3", 0.0);
  map.add("layer4", 0.0);
  map.add("layer5", 0.0);
  map.add("layer6", 0.0);

  std::cout << "Results for iteration over " << map.getSize()(0) << " x " << map.getSize()(1) <<
    " (" <<
    map.getSize().prod() << ") grid cells." << std::endl;
  std::cout << "=========================================" << std::endl;

  clk::time_point t1 = clk::now();
  runGridMapIteratorVersion1(map, "random", "layer1");
  clk::time_point t2 = clk::now();
  std::cout << "Duration grid map iterator (convenient use): " << duration(t2 - t1) << " ms" <<
    std::endl;

  t1 = clk::now();
  runGridMapIteratorVersion2(map, "random", "layer2");
  t2 = clk::now();
  std::cout << "Duration grid map iterator (direct access to data layers): " << duration(t2 - t1) <<
    " ms" << std::endl;

  t1 = clk::now();
  runGridMapIteratorVersion3(map, "random", "layer3");
  t2 = clk::now();
  std::cout << "Duration grid map iterator (linear grid_map::Index): " << duration(t2 - t1) <<
    " ms" << std::endl;

  t1 = clk::now();
  runEigenFunction(map, "random", "layer4");
  t2 = clk::now();
  std::cout << "Duration Eigen function: " << duration(t2 - t1) << " ms" << std::endl;

  t1 = clk::now();
  runCustomIndexIteration(map, "random", "layer5");
  t2 = clk::now();
  std::cout << "Duration custom grid_map::Index iteration: " << duration(t2 - t1) << " ms" <<
    std::endl;

  t1 = clk::now();
  runCustomLinearIndexIteration(map, "random", "layer6");
  t2 = clk::now();
  std::cout << "Duration custom linear grid_map::Index iteration: " << duration(t2 - t1) << " ms" <<
    std::endl;

  return 0;
}
