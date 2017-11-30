/*
 * ColorFillFilter.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <grid_map_filters/ColorFillFilter.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>

using namespace filters;

namespace grid_map {

template<typename T>
ColorFillFilter<T>::ColorFillFilter()
    : r_(0.0),
      g_(0.0),
      b_(0.0)
{
}

template<typename T>
ColorFillFilter<T>::~ColorFillFilter()
{
}

template<typename T>
bool ColorFillFilter<T>::configure()
{
  if (!FilterBase < T > ::getParam(std::string("red"), r_)) {
    ROS_ERROR("Color fill filter did not find parameter `red`.");
    return false;
  }
  ROS_DEBUG("Color fill filter red is = %f.", r_);

  if (!FilterBase < T > ::getParam(std::string("green"), g_)) {
    ROS_ERROR("Color fill filter did not find parameter `green`.");
    return false;
  }
  ROS_DEBUG("Color fill filter green is = %f.", g_);

  if (!FilterBase < T > ::getParam(std::string("blue"), b_)) {
    ROS_ERROR("Color fill filter did not find parameter `blue`.");
    return false;
  }
  ROS_DEBUG("Color fill filter blue is = %f.", b_);

  if (!FilterBase < T > ::getParam(std::string("mask_layer"), maskLayer_));
  ROS_DEBUG("Color fill filter mask_layer = %s.", maskLayer_.c_str());

  if (!FilterBase < T > ::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Color fill filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Color fill filter output_layer = %s.", outputLayer_.c_str());
  return true;
}

template<typename T>
bool ColorFillFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  const Eigen::Vector3f colorVector(r_, g_, b_);
  float colorValue;
  colorVectorToValue(colorVector, colorValue);

  if (maskLayer_.empty()) {
    mapOut.add(outputLayer_, colorValue);

  } else {
    mapOut.add(outputLayer_);
    auto& output = mapOut[outputLayer_];
    auto& mask = mapOut[maskLayer_];

    // For each cell in map.
    for (size_t i = 0; i < output.size(); ++i) {
      output(i) = std::isfinite(mask(i)) ? colorValue : NAN;
    }
  }
  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::ColorFillFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
