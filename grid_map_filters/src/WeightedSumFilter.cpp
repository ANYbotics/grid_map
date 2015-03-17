/*
 * WeightedSumFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/WeightedSumFilter.hpp"
#include <pluginlib/class_list_macros.h>

#include <vector>
#include <cstring>
#include <string>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/GridMap.hpp>

namespace filters {

template<typename T>
WeightedSumFilter<T>::WeightedSumFilter()
      : traversabilityType_("traversability")
{

}

template<typename T>
WeightedSumFilter<T>::~WeightedSumFilter()
{

}

template<typename T>
bool WeightedSumFilter<T>::configure()
{
  // Load Parameters
  if (!FilterBase<T>::getParam(std::string("filterTypes"), additionTypes_)) {
    ROS_ERROR("WeightedSumFilter did not find param filterTypes");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("filterWeights"), additionWeights_)) {
    ROS_ERROR("WeightedSumFilter did not find param filterWeights");
    return false;
  }

  if (additionTypes_.size() != additionWeights_.size()) {
    ROS_ERROR("Number of weights must correspond number of layers!");
    return false;
  }

  // Normalize weights
  double sumWeights = 0.0;
  for (std::vector<double>::iterator it=additionWeights_.begin(); it!=additionWeights_.end(); ++it) {
//    ROS_INFO("Weight = %f", *it);
    sumWeights += *it;
  }

//  ROS_INFO("Sum weights = %f", sumWeights);
  for (std::vector<double>::iterator it=additionWeights_.begin(); it!=additionWeights_.end(); ++it) {
    *it /= sumWeights;
//    ROS_INFO("Weight = %f", *it);
  }
  return true;
}

template<typename T>
bool WeightedSumFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(traversabilityType_, mapIn.get("elevation"));

  for (std::vector<std::string>::iterator it=additionTypes_.begin(); it!=additionTypes_.end(); ++it) {
    if (!mapOut.exists(*it)) {
//      ROS_ERROR("Check your addition types! Type %s does not exist",*it.c_str());
    }
  }


  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(WeightedSumFilter, filters::WeightedSumFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
