/*
 * WeightedSumFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/WeightedSumFilter.hpp"
#include <pluginlib/class_list_macros.h>

namespace filters {

template<typename T>
WeightedSumFilter<T>::WeightedSumFilter()
{

}

template<typename T>
WeightedSumFilter<T>::~WeightedSumFilter()
{

}

template<typename T>
bool WeightedSumFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("filterTypes"), additionTypes_)) {
    ROS_ERROR("WeightedSumFilter did not find param filterTypes");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("filterWeights"), additionWeights_)) {
    ROS_ERROR("WeightedSumFilter did not find param filterWeights");
    return false;
  }
  return true;
}

template<typename T>
bool WeightedSumFilter<T>::update(const T& elevation_map, T& traversability_map)
{
  traversability_map = elevation_map;
  ROS_INFO("Number of addition types = %d",additionTypes_.size());
  ROS_INFO("Number of addition weights = %d",additionWeights_.size());
  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(WeightedSumFilter, filters::WeightedSumFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
