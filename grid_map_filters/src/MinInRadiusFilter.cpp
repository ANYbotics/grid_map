/*
 * MinInRadiusFilter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "filters/MinInRadiusFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map_filters {

template<typename T>
MinInRadiusFilter<T>::MinInRadiusFilter()
    : radius_(0.02),
	  inputLayer_("footscore"),
      type_("minInRadius")
{

}

template<typename T>
MinInRadiusFilter<T>::~MinInRadiusFilter()
{

}

template<typename T>
bool MinInRadiusFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("MinInRadius filter did not find param radius.");
    return false;
  }

  if (radius_ < 0.0) {
    ROS_ERROR("Radius must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Radius = %f.", radius_);


  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
      ROS_ERROR("MinInRadius filter did not find param input_layer.");
      return false;
    }

  ROS_DEBUG("MinInRadius input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("Step filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("MinInRadius map type = %s.", type_.c_str());

  return true;
}

template<typename T>
bool MinInRadiusFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  double value;

  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, inputLayer_))
      continue;
    value = mapOut.at(inputLayer_, *iterator);
    double valueMin;

    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Get minimal value in the circular window.
    bool init = false;
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius_);
        !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, inputLayer_))
        continue;
      value = mapOut.at(inputLayer_, *submapIterator);

      if (!init) {
        valueMin = value;
        init = true;
        continue;
      }
      if (value < valueMin)
        valueMin = value;
    }

    if (init)
      mapOut.at(type_, *iterator) = valueMin;
  }

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(MinInRadiusFilter, grid_map_filters::MinInRadiusFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
