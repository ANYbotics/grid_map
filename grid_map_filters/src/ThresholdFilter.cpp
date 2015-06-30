/*
 * ThresholdFilter.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/ThresholdFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map lib
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

// ROS
#include <ros/ros.h>

using namespace filters;

namespace grid_map_filters {

template<typename T>
ThresholdFilter<T>::ThresholdFilter()
    : useLowerThreshold_(false),
      useUpperThreshold_(false),
      lowerThreshold_(0.0),
      upperThreshold_(1.0),
      setTo_(0.5)
{

}

template<typename T>
ThresholdFilter<T>::~ThresholdFilter()
{

}

template<typename T>
bool ThresholdFilter<T>::configure()
{
  // Load Parameters
  if (FilterBase<T>::getParam(std::string("lower_threshold"),
                              lowerThreshold_)) {
    useLowerThreshold_ = true;
    ROS_INFO("lower threshold = %f", lowerThreshold_);
  }

  if (FilterBase<T>::getParam(std::string("upper_threshold"),
                              upperThreshold_)) {
    useUpperThreshold_ = true;
    ROS_INFO("upper threshold = %f", upperThreshold_);
  }

  if (!useLowerThreshold_ && !useUpperThreshold_) {
    ROS_ERROR(
        "ThresholdFilter did not find parameter 'lower_threshold' or 'upper_threshold',");
    return false;
  }

  if (useLowerThreshold_ && useUpperThreshold_) {
    ROS_ERROR(
        "Set either 'lower_threshold' or 'upper_threshold'! Only one threshold can be used!");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("set_to"), setTo_)) {
    ROS_ERROR("ThresholdFilter did not find parameter 'set_to'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("layers"), layers_)) {
    ROS_ERROR("ThresholdFilter did not find parameter 'layers'.");
    return false;
  }

  return true;
}

template<typename T>
bool ThresholdFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;

  for (const auto& layer : layers_) {
    // Check if layer exists.
    if (!mapOut.exists(layer)) {
      ROS_ERROR("Check your threshold types! Type %s does not exist",
                layer.c_str());
      continue;
    }

    std::vector<std::string> validTypes;
    validTypes.push_back(layer);

    for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd();
        ++iterator) {
      if (!mapOut.isValid(*iterator, validTypes))
        continue;

      double value = mapOut.at(layer, *iterator);
      if (useLowerThreshold_) {
        if (value < lowerThreshold_)
          value = setTo_;
      }
      if (useUpperThreshold_) {
        if (value > upperThreshold_)
          value = setTo_;
      }
      mapOut.at(layer, *iterator) = value;
    }

  }

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(ThresholdFilter,
                         grid_map_filters::ThresholdFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
