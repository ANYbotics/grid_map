/*
 * MeanInRadiusFilter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MeanInRadiusFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
MeanInRadiusFilter<T>::MeanInRadiusFilter()
: radius_(0.0)
{
}

template<typename T>
MeanInRadiusFilter<T>::~MeanInRadiusFilter()
{
}

template<typename T>
bool MeanInRadiusFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("radius"), radius_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "MeanInRadius filter did not find parameter `radius`.");
    return false;
  }

  if (radius_ < 0.0) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "MeanInRadius filter: Radius must be greater than zero.");
    return false;
  }

  RCLCPP_DEBUG(this->logging_interface_->get_logger(), "Radius = %f.", radius_);

  if (!param_reader.get(std::string("input_layer"), inputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "MeanInRadius filter did not find parameter `input_layer`.");
    return false;
  }

  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "MeanInRadius input layer is = %s.",
    inputLayer_.c_str());

  if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "MeanInRadius filter did not find parameter `output_layer`.");
    return false;
  }

  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "MeanInRadius output_layer = %s.",
    outputLayer_.c_str());
  return true;
}

template<typename T>
bool MeanInRadiusFilter<T>::update(const T & mapIn, T & mapOut)
{
  // Add new layers to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  double value;

  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    double valueSum = 0.0;
    int counter = 0;
    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Find the mean in a circle around the center
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius_);
      !submapIterator.isPastEnd();
      ++submapIterator)
    {
      if (!mapOut.isValid(*submapIterator, inputLayer_)) {
        continue;
      }
      value = mapOut.at(inputLayer_, *submapIterator);
      valueSum += value;
      counter++;
    }

    if (counter != 0) {
      mapOut.at(outputLayer_, *iterator) = valueSum / counter;
    }
  }

  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::MeanInRadiusFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
