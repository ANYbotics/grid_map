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
    ROS_ERROR("ThresholdFilter did not find param sumType");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("upperThreshold"), upperThreshold_)) {
    ROS_ERROR("ThresholdFilter did not find param upperThreshold");
    return false;
  }

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

  for (int i=0; i<thresholdTypes_.size(); i++) {
    if (!mapOut.exists(thresholdTypes_.at(i))) {
      ROS_ERROR("Check your threshold types! Type %s does not exist",thresholdTypes_.at(i).c_str());
      continue;
    }

    std::vector<std::string> validTypes;
    validTypes.push_back(thresholdTypes_.at(i));

    for (grid_map_lib::GridMapIterator iterator(mapOut); !iterator.isPassedEnd(); ++iterator) {
      if (!mapOut.isValid(*iterator, validTypes)) continue;

      //TODO Define Threshold function
    }


  }

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(ThresholdFilter, filters::ThresholdFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
