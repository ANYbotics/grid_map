/*
 * SetBasicLayersFilters.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/SetBasicLayersFilter.hpp"

#include <grid_map_core/GridMap.hpp>

using namespace filters;

namespace grid_map {

SetBasicLayersFilter::SetBasicLayersFilter() = default;

SetBasicLayersFilter::~SetBasicLayersFilter() = default;

bool SetBasicLayersFilter::configure() {
  if (!FilterBase::getParam(std::string("layers"), layers_)) {
    ROS_ERROR("SetBasicLayersFilters did not find parameter 'layers'.");
    return false;
  }

  return true;
}

bool SetBasicLayersFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  std::vector<std::string> layersChecked;

  for (const auto& layer : layers_) {
    if (!mapOut.exists(layer)) {
      ROS_WARN("Layer `%s` does not exist and is not set as basic layer.", layer.c_str());
      continue;
    }
    layersChecked.push_back(layer);
  }

  mapOut.setBasicLayers(layersChecked);
  return true;
}

}  // namespace grid_map
