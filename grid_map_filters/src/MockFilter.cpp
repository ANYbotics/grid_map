/*
 * MockFilter.cpp
 *
 *  Created on: Sep 24, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <chrono>

#include "grid_map_filters/MockFilter.hpp"

using namespace filters;

namespace grid_map {

template<typename T>
MockFilter<T>::MockFilter()
{
}

template<typename T>
MockFilter<T>::~MockFilter()
{
}

template<typename T>
bool MockFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("processing_time"), processingTime_)) {
    ROS_ERROR("MockFilter did not find parameter 'processing_time'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("print_name"), printName_)) {
    ROS_ERROR("MockFilter did not find parameter 'print_name'.");
    return false;
  }

  return true;
}

template<typename T>
bool MockFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  if(printName_){
    ROS_INFO_STREAM(this->getName()<<": update()");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(processingTime_));
  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::MockFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
