/*
 * DuplicationFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/DuplicationFilter.hpp"

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

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
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("input_layer"), inputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "DuplicationFilter did not find parameter 'input_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "DuplicationFilter did not find parameter 'output_layer'.");
    return false;
  }

  return true;
}

template<typename T>
bool DuplicationFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;
  mapOut.add(outputLayer_, mapIn[inputLayer_]);
  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::DuplicationFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
