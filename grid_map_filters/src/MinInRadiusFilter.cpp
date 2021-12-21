/*
 * MinInRadiusFilter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MinInRadiusFilter.hpp"

#include <math.h>
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

MinInRadiusFilter::MinInRadiusFilter() : radius_(0.0) {}

MinInRadiusFilter::~MinInRadiusFilter() = default;

bool MinInRadiusFilter::configure() {
  if (!FilterBase::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("MinInRadius filter did not find parameter `radius`.");
    return false;
  }

  if (radius_ < 0.0) {
    ROS_ERROR("MinInRadius filter: Radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("Radius = %f.", radius_);

  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("MinInRadius filter did not find parameter `input_layer`.");
    return false;
  }

  ROS_DEBUG("MinInRadius input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Step filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("MinInRadius output_layer = %s.", outputLayer_.c_str());
  return true;
}

bool MinInRadiusFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  double value{NAN};

  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, inputLayer_)) {
      continue;
    }
    value = mapOut.at(inputLayer_, *iterator);
    double valueMin = 0.0;

    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Get minimal value in the circular window.
    bool init = false;
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius_); !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, inputLayer_)) {
        continue;
      }
      value = mapOut.at(inputLayer_, *submapIterator);

      if (!init) {
        valueMin = value;
        init = true;
        continue;
      }
      if (value < valueMin) {
        valueMin = value;
      }
    }

    if (init) {
      mapOut.at(outputLayer_, *iterator) = valueMin;
    }
  }

  return true;
}

}  // namespace grid_map
