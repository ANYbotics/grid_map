/*
 * DeletionFilter.cpp
 *
 *  Created on: Mar 19, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/DeletionFilter.hpp"

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
DeletionFilter<T>::DeletionFilter()
{
}

template<typename T>
DeletionFilter<T>::~DeletionFilter()
{
}

template<typename T>
bool DeletionFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  // Load Parameters
  if (!param_reader.get(std::string("layers"), layers_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(), "DeletionFilter did not find parameter 'layers'.");
    return false;
  }

  return true;
}

template<typename T>
bool DeletionFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;

  for (const auto & layer : layers_) {
    // Check if layer exists.
    if (!mapOut.exists(layer)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "Check your deletion layers! Type %s does not exist.",
        layer.c_str());
      continue;
    }

    if (!mapOut.erase(layer)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(), "Could not remove type %s.",
        layer.c_str());
    }
  }

  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::DeletionFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
