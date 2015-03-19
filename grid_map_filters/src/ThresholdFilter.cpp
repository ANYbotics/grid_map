/*
 * ThresholdFilter.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/ThresholdFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/GridMap.hpp>
#include <grid_map_lib/iterators/GridMapIterator.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

namespace filters {

template<typename T>
ThresholdFilter<T>::ThresholdFilter()
    : lowerThreshold_(0.1),
      upperThreshold_(0.9)
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
  if (!FilterBase<T>::getParam(std::string("lowerThreshold"), lowerThreshold_)) {
    ROS_ERROR("ThresholdFilter did not find param lowerThreshold");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("upperThreshold"), upperThreshold_)) {
    ROS_ERROR("ThresholdFilter did not find param upperThreshold");
    return false;
  }

  if (lowerThreshold_ < 0.0) {
    ROS_ERROR("The lower threshold must be greater than zero");
    return false;
  }

  if (upperThreshold_ > 1.0) {
    ROS_ERROR("The upper threshold must be smaller than 1.0");
    return false;
  }

  if (lowerThreshold_ > upperThreshold_) {
    ROS_ERROR("The upper threshold must be greater than the lower threshold");
    return false;
  }

  ROS_INFO("lower threshold = %f", lowerThreshold_);
  ROS_INFO("upper threshold = %f", upperThreshold_);

  if (!FilterBase<T>::getParam(std::string("thresholdTypes"), thresholdTypes_)) {
    ROS_ERROR("ThresholdFilter did not find param thresholdTypes");
    return false;
  }

  return true;
}

template<typename T>
bool ThresholdFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;

  for (int i = 0; i < thresholdTypes_.size(); i++) {
    // Check if layer exists.
    if (!mapOut.exists(thresholdTypes_.at(i))) {
      ROS_ERROR("Check your threshold types! Type %s does not exist",
                thresholdTypes_.at(i).c_str());
      continue;
    }

    std::vector<std::string> validTypes;
    validTypes.push_back(thresholdTypes_.at(i));

    for (grid_map_lib::GridMapIterator iterator(mapOut);
        !iterator.isPassedEnd(); ++iterator) {
      if (!mapOut.isValid(*iterator, validTypes))
        continue;

      double value = mapOut.at(thresholdTypes_.at(i), *iterator);
      if (value > upperThreshold_)
        value = 1.0;
      if (value < lowerThreshold_)
        value = 0.0;
      mapOut.at(thresholdTypes_.at(i), *iterator) = value;
    }

  }

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(ThresholdFilter, filters::ThresholdFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
