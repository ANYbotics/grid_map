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

void overwriteGridMapIterator(GridMap& map, const string& layer_from, const string layer_to)
{
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    float value = map.at(layer_from, *iterator);
    if (isnan(value)) continue;
    map.at(layer_to, *iterator) = value;
  }
}

void overwriteGridMapLayerDataIterator(GridMap& map, const string& layer_from,
                                       const string layer_to)
{
  auto& m_to = map[layer_to];
  for (GridMapLayerDataIterator iterator(map, layer_from); !iterator.isPastEnd(); ++iterator) {
    const float& value = iterator.getValue();
    if (isnan(value)) continue;
    m_to(iterator.getLinearIndex()) = value;
  }
}

void overwriteLayerCustom(GridMap& map, const string& layer_from, const string layer_to)
{
  const auto& m_from = map[layer_from];
  auto& m_to = map[layer_to];
  for (int j = 0; j < m_to.cols(); ++j) {
    for (int i = 0; i < m_to.rows(); ++i) {
      auto value = m_from(i, j);
      if (isnan(value)) continue;
      m_to(i, j) = value;
    }
  }
}

int main(int argc, char* argv[])
{
  GridMap map;
  map.setGeometry(Length(20.0, 20.0), 0.01, Position(0.0, 0.0));
  map.add("layer", 0.0);
  map.add("layer2", 2.0);
  map.add("layer3", 3.0);
  map.add("layer4", 4.0);

  clk::time_point t1 = clk::now();
  overwriteGridMapIterator(map, "layer2", "layer");
  clk::time_point t2 = clk::now();
  std::cout << "Duration grid map iterator: " << duration(t2 - t1) << " ms" << std::endl;

  t1 = clk::now();
  overwriteGridMapLayerDataIterator(map, "layer3", "layer");
  t2 = clk::now();
  std::cout << "Duration grid map layer data iterator: " << duration(t2 - t1) << " ms" << std::endl;

  t1 = clk::now();
  overwriteLayerCustom(map, "layer4", "layer");
  t2 = clk::now();
  std::cout << "Duration custom iteration: " << duration(t2 - t1) << " ms" <<  std::endl;

  return 0;
}
