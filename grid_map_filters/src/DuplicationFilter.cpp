/*
 * DuplicationFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/DuplicationFilter.hpp"

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

template<typename T>
DuplicationFilter<T>::DuplicationFilter()
{
}

template<typename T>
DuplicationFilter<T>::~DuplicationFilter()
{
}

template<typename T>
bool DuplicationFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("DuplicationFilter did not find parameter 'input_layer'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("DuplicationFilter did not find parameter 'output_layer'.");
    return false;
  }

  return true;
}

template<typename T>
bool DuplicationFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(outputLayer_, mapIn[inputLayer_]);
  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::DuplicationFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
