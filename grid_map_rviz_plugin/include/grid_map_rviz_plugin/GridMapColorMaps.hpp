/*
 * GridMapColorMaps.hpp
 *
 *  Created on: Apr 27, 2021
 *  Author: Matias Mattamala
 *  Institute: University of Oxford
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <OGRE/OgreMaterial.h>

namespace grid_map_rviz_plugin {

extern std::map<std::string, std::vector<std::vector<float>>> const colorMap;

inline Ogre::ColourValue getColorMap(float intensity, const std::vector<std::vector<float>>& ctable) {
  intensity = std::min(intensity, 1.0f);
  intensity = std::max(intensity, 0.0f);

  int idx = int(floor(intensity*255));
  idx = std::min(idx, 255);
  idx = std::max(idx, 0);

  // Get color from table
  std::vector<float> const& rgb = ctable.at(idx);

  Ogre::ColourValue color;
  color.r = rgb[0];
  color.g = rgb[1];
  color.b = rgb[2];

  return color;
}

inline std::vector<std::string> getColorMapNames() {
  std::vector<std::string> types;
  types.reserve(colorMap.size());

  for(auto const& pair : colorMap) {
    types.push_back(pair.first);
  }
  
  return types;
}

}  // end namespace grid_map_rviz_plugin