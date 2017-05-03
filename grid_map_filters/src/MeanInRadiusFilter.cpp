/*
 * MeanInRadiusFilter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "filters/MeanInRadiusFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map_filters {

template<typename T>
MeanInRadiusFilter<T>::MeanInRadiusFilter()
    : radius_(0.02),
	  inputLayer_("elevation"),
      type_("meanInRadius")
{

}

template<typename T>
MeanInRadiusFilter<T>::~MeanInRadiusFilter()
{

}

template<typename T>
bool MeanInRadiusFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("Step filter did not find param radius.");
    return false;
  }

  if (radius_ < 0.0) {
    ROS_ERROR("Radius must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Radius = %f.", radius_);


  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
      ROS_ERROR("Step filter did not find param input_layer.");
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
bool MeanInRadiusFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layers to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  double value;

  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {

    double valueSum = 0.0;
    int counter = 0;
    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Find the mean in a circle around the center
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius_);
        !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, inputLayer_))
        continue;
      value = mapOut.at(inputLayer_, *submapIterator);
      // Init heightMax and heightMin
      valueSum+=value;
      counter ++;
    }

    if (counter!=0)
      mapOut.at(type_, *iterator) = valueSum/counter;
  }

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(MeanInRadiusFilter, grid_map_filters::MeanInRadiusFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
