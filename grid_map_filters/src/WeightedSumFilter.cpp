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
  return true;
}

template<typename T>
bool WeightedSumFilter<T>::update(const T& elevation_map, T& traversability_map)
{
  traversability_map = elevation_map;
  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(WeightedSumFilter, filters::WeightedSumFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
