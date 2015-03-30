/*
 * MinFilter.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/MinFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map lib
#include <grid_map_core/GridMap.hpp>

using namespace filters;

namespace grid_map_filters {

template<typename T>
MinFilter<T>::MinFilter()
    : layerOut_("traversability")
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
  if (!FilterBase<T>::getParam(std::string("layer_out"), layerOut_)) {
    ROS_ERROR("MinFilter did not find parameter 'layer_out'.");
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
  mapOut.add(layerOut_);
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
  mapOut.add(layerOut_, minMatrix);

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(MinFilter,
                         grid_map_filters::MinFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
