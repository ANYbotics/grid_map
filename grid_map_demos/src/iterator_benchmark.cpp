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
void runGridMapIteratorVersion1(GridMap& map, const string& layer_from, const string layer_to)
{
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    float value_from = map.at(layer_from, *iterator);
    float& value_to = map.at(layer_to, *iterator);
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Use this setup for improved efficiency when handling big maps.
 */
void runGridMapIteratorVersion2(GridMap& map, const string& layer_from, const string layer_to)
{
  const auto& map_from = map[layer_from];
  auto& map_to = map[layer_to];
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const float value_from = map_from(iterator.getLinearIndex());
    float& value_to = map_to(iterator.getLinearIndex());
    value_to = value_to > value_from ? value_to : value_from;
  }
}

/*!
 * Whenever possible, make use of the Eigen methods for maximum efficiency
 * and readability.
 */
void runEigenFunction(GridMap& map, const string& layer_from, const string layer_to)
{
  map[layer_to] = map[layer_to].cwiseMax(map[layer_from]);
}

/*!
 * For comparison.
 */
void runCustomIndexIteration(GridMap& map, const string& layer_from, const string layer_to)
{
  const auto& m_from = map[layer_from];
  auto& m_to = map[layer_to];
  for (int j = 0; j < m_to.cols(); ++j) {
    for (int i = 0; i < m_to.rows(); ++i) {
      const auto value = m_from(i, j);
      m_to(i, j) = m_to(i, j) > value ? m_to(i, j) : value;
    }
  }
}

/*!
 * For comparison.
 */
void runCustomLinearIndexIteration(GridMap& map, const string& layer_from, const string layer_to)
{
  const auto& m_from = map[layer_from];
  auto& m_to = map[layer_to];
  for (int i = 0; i < m_to.size(); ++i) {
    m_to(i) = m_to(i) > m_from(i) ? m_to(i) : m_from(i);
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

  cout << "Results for iteration over " << map.getSize()(0) << " x " << map.getSize()(1) << " (" << map.getSize().prod() << ") grid cells." << endl;
  cout << "=========================================" << endl;

  clk::time_point t1 = clk::now();
  runGridMapIteratorVersion1(map, "random", "layer1");
  clk::time_point t2 = clk::now();
  cout << "Duration grid map iterator (version 1): " << duration(t2 - t1) << " ms" << endl;

  t1 = clk::now();
  runGridMapIteratorVersion2(map, "random", "layer2");
  t2 = clk::now();
  cout << "Duration grid map iterator (version 2): " << duration(t2 - t1) << " ms" << endl;

  t1 = clk::now();
  runEigenFunction(map, "random", "layer3");
  t2 = clk::now();
  cout << "Duration Eigen function: " << duration(t2 - t1) << " ms" <<  endl;

  t1 = clk::now();
  runCustomIndexIteration(map, "random", "layer4");
  t2 = clk::now();
  cout << "Duration custom index iteration: " << duration(t2 - t1) << " ms" <<  endl;

  t1 = clk::now();
  runCustomLinearIndexIteration(map, "random", "layer5");
  t2 = clk::now();
  cout << "Duration custom linear index iteration: " << duration(t2 - t1) << " ms" <<  endl;

  return 0;
}
