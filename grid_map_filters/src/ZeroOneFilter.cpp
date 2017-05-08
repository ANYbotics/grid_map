/*
 * ZeroOneFilter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "filters/ZeroOneFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map_filters {

template<typename T>
ZeroOneFilter<T>::ZeroOneFilter()
    : threshold_(0.8),
	  inputLayer_("footscore"),
      type_("minInRadius")
{

}

template<typename T>
ZeroOneFilter<T>::~ZeroOneFilter()
{

}

template<typename T>
bool ZeroOneFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("threshold"), threshold_)) {
    ROS_ERROR("ZeroOne filter did not find param threshold.");
    return false;
  }

  if (threshold_ < 0.0 ||threshold_ >1.0) {
    ROS_ERROR("Threshold must lie between one and zero.");
    return false;
  }

  ROS_DEBUG("Threshold = %f.", threshold_);


  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
      ROS_ERROR("ZeroOne filter did not find param input_layer.");
      return false;
    }

  ROS_DEBUG("ZeroOne input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("ZeroOne filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("MinInRadius map type = %s.", type_.c_str());

  return true;
}

template<typename T>
bool ZeroOneFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  double value;

  // Iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, inputLayer_))
      continue;
    value = mapOut.at(inputLayer_, *iterator);

    if (value >= threshold_){
    	mapOut.at(type_, *iterator) = 1.0;
    }else{
    	mapOut.at(type_, *iterator) = 0.0;
    }
  }

  return true;
}


}/* namespace */

PLUGINLIB_REGISTER_CLASS(ZeroOneFilter, grid_map_filters::ZeroOneFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
