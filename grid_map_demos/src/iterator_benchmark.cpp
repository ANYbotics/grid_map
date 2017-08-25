/*
 * grid_map_iterator_benchmark.hpp
 *
 *  Created on: Feb 15, 2016
 *     Authors: Christos Zalidis, Péter Fankhauser
 */

#include <grid_map_core/grid_map_core.hpp>
#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono;
using namespace grid_map;

#define duration(a) duration_cast<milliseconds>(a).count()
typedef high_resolution_clock clk;

/*!
 * Convenient use of iterator.
 */
void runGridMapIteratorVersion1(GridMap& map, const string& layer_from, const string& layer_to)
{
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const float value_from = map.at(layer_from, *iterator);
    float& value_to = map.at(layer_to, *iterator);
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Improved efficiency by storing direct access to data layers.
 */
void runGridMapIteratorVersion2(GridMap& map, const string& layer_from, const string& layer_to)
{
  const auto& data_from = map[layer_from];
  auto& data_to = map[layer_to];
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    const float value_from = data_from(index(0), index(1));
    float& value_to = data_to(index(0), index(1));
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Improved efficiency by using linear index.
 */
void runGridMapIteratorVersion3(GridMap& map, const string& layer_from, const string& layer_to)
{
  const auto& data_from = map[layer_from];
  auto& data_to = map[layer_to];
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const size_t i = iterator.getLinearIndex();
    const float value_from = data_from(i);
    float& value_to = data_to(i);
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Whenever possible, make use of the Eigen methods for maximum efficiency
 * and readability.
 */
void runEigenFunction(GridMap& map, const string& layer_from, const string& layer_to)
{
  map[layer_to] = map[layer_to].cwiseMax(map[layer_from]);
}

/*!
 * For comparison.
 */
void runCustomIndexIteration(GridMap& map, const string& layer_from, const string& layer_to)
{
  const auto& data_from = map[layer_from];
  auto& data_to = map[layer_to];
  for (size_t j = 0; j < data_to.cols(); ++j) {
    for (size_t i = 0; i < data_to.rows(); ++i) {
      const float value_from = data_from(i, j);
      float& value_to = data_to(i, j);
      value_to = value_to > value_from ? value_to : value_from;
    }
  }
}

/*!
 * For comparison.
 */
void runCustomLinearIndexIteration(GridMap& map, const string& layer_from, const string& layer_to)
{
  const auto& data_from = map[layer_from];
  auto& data_to = map[layer_to];
  for (size_t i = 0; i < data_to.size(); ++i) {
    data_to(i) = data_to(i) > data_from(i) ? data_to(i) : data_from(i);
  }
}

int main(int argc, char* argv[])
{
  GridMap map;
  map.setGeometry(Length(20.0, 20.0), 0.004, Position(0.0, 0.0));
  map.add("random");
  map["random"].setRandom();
  map.add("layer1", 0.0);
  map.add("layer2", 0.0);
  map.add("layer3", 0.0);
  map.add("layer4", 0.0);
  map.add("layer5", 0.0);
  map.add("layer6", 0.0);

  cout << "Results for iteration over " << map.getSize()(0) << " x " << map.getSize()(1) << " (" << map.getSize().prod() << ") grid cells." << endl;
  cout << "=========================================" << endl;

  clk::time_point t1 = clk::now();
  runGridMapIteratorVersion1(map, "random", "layer1");
  clk::time_point t2 = clk::now();
  cout << "Duration grid map iterator (convenient use): " << duration(t2 - t1) << " ms" << endl;

  t1 = clk::now();
  runGridMapIteratorVersion2(map, "random", "layer2");
  t2 = clk::now();
  cout << "Duration grid map iterator (direct access to data layers): " << duration(t2 - t1) << " ms" << endl;

  t1 = clk::now();
  runGridMapIteratorVersion3(map, "random", "layer3");
  t2 = clk::now();
  cout << "Duration grid map iterator (linear index): " << duration(t2 - t1) << " ms" << endl;

  t1 = clk::now();
  runEigenFunction(map, "random", "layer4");
  t2 = clk::now();
  cout << "Duration Eigen function: " << duration(t2 - t1) << " ms" <<  endl;

  t1 = clk::now();
  runCustomIndexIteration(map, "random", "layer5");
  t2 = clk::now();
  cout << "Duration custom index iteration: " << duration(t2 - t1) << " ms" <<  endl;

  t1 = clk::now();
  runCustomLinearIndexIteration(map, "random", "layer6");
  t2 = clk::now();
  cout << "Duration custom linear index iteration: " << duration(t2 - t1) << " ms" <<  endl;

  return 0;
}
