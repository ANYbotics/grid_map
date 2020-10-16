/*
 * SetBasicLayersFilters.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "../include/grid_map_filters/SetBasicLayersFilter.hpp"

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <vector>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
SetBasicLayersFilter<T>::SetBasicLayersFilter()
{
}

template<typename T>
SetBasicLayersFilter<T>::~SetBasicLayersFilter()
{
}

template<typename T>
bool SetBasicLayersFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("layer"), layers_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "SetBasicLayersFilters did not find parameter 'layers'.");
    return false;
  }

  return true;
}

template<typename T>
bool SetBasicLayersFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;
  std::vector<std::string> layersChecked;

  for (const auto & layer : layers_) {
    if (!mapOut.exists(layer)) {
      RCLCPP_WARN(
        this->logging_interface_->get_logger(),
        "Layer `%s` does not exist and is not set as basic layer.",
        layer.c_str());
      continue;
    }
    layersChecked.push_back(layer);
  }

  mapOut.setBasicLayers(layersChecked);
  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::SetBasicLayersFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
