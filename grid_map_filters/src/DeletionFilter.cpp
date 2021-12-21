/*
 * DeletionFilter.cpp
 *
 *  Created on: Mar 19, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/DeletionFilter.hpp"

#include <grid_map_core/GridMap.hpp>

using namespace filters;

namespace grid_map {

DeletionFilter::DeletionFilter() = default;

DeletionFilter::~DeletionFilter() = default;

bool DeletionFilter::configure() {
  // Load Parameters
  if (!FilterBase::getParam(std::string("layers"), layers_)) {
    ROS_ERROR("DeletionFilter did not find parameter 'layers'.");
    return false;
  }

  return true;
}

bool DeletionFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;

  for (const auto& layer : layers_) {
    // Check if layer exists.
    if (!mapOut.exists(layer)) {
      ROS_ERROR("Check your deletion layers! Type %s does not exist.", layer.c_str());
      continue;
    }

    if (!mapOut.erase(layer)) {
      ROS_ERROR("Could not remove type %s.", layer.c_str());
    }
  }

  return true;
}

}  // namespace grid_map
