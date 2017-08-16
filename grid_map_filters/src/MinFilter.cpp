/*
 * MinFilter.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_filters/MinFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

template<typename T>
MinFilter<T>::MinFilter()
{
}

template<typename T>
MinFilter<T>::~MinFilter()
{
}

template<typename T>
bool MinFilter<T>::configure()
{
  // Load Parameters
  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("MinFilter did not find parameter 'output_layer'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("layers"), layers_)) {
    ROS_ERROR("MinFilter did not find parameter 'layers'.");
    return false;
  }

  return true;
}

template<typename T>
bool MinFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(outputLayer_);
  bool hasMin = false;

  grid_map::Matrix minMatrix;

  for (const auto& layer : layers_) {
    // Check if layer exists.
    if (!mapOut.exists(layer)) {
      ROS_ERROR("Check your min layers! Type %s does not exist.", layer.c_str());
      return false;
    }

    if (!hasMin) {
      minMatrix = mapOut.get(layer);
      hasMin = true;
    } else {
      minMatrix = minMatrix.cwiseMin(mapOut.get(layer));
    }
  }
  mapOut.add(outputLayer_, minMatrix);

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(MinFilter, grid_map::MinFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
