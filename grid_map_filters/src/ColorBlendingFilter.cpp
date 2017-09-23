/*
 * ColorBlendingFilter.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <grid_map_filters/ColorBlendingFilter.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>
#include <math.h>

using namespace filters;

namespace grid_map {

template<typename T>
ColorBlendingFilter<T>::ColorBlendingFilter()
    : opacity_(0.0)
{
}

template<typename T>
ColorBlendingFilter<T>::~ColorBlendingFilter()
{
}

template<typename T>
bool ColorBlendingFilter<T>::configure()
{
  if (!FilterBase < T > ::getParam(std::string("background_layer"), backgroundLayer_)) {
    ROS_ERROR("Color blending filter did not find parameter `background_layer`.");
    return false;
  }
  ROS_DEBUG("Color blending filter background layer is = %s.", backgroundLayer_.c_str());

  if (!FilterBase < T > ::getParam(std::string("foreground_layer"), foregroundLayer_)) {
    ROS_ERROR("Color blending filter did not find parameter `foreground_layer`.");
    return false;
  }
  ROS_DEBUG("Color blending filter foreground layer is = %s.", foregroundLayer_.c_str());

  std::string blendMode;
  if (!FilterBase < T > ::getParam(std::string("blend_mode"), blendMode)) {
    blendMode = "normal";
  }
  ROS_DEBUG("Color blending filter blend mode is = %s.", blendMode.c_str());
  if (blendMode == "normal") blendMode_ = BlendModes::Normal;
  else if (blendMode == "soft_light") blendMode_ = BlendModes::SoftLight;
  else {
    ROS_ERROR("Color blending filter blend mode `%s` does not exist.", blendMode.c_str());
    return false;
  }

  if (!FilterBase < T > ::getParam(std::string("opacity"), opacity_)) {
    ROS_ERROR("Color blending filter did not find parameter `opacity`.");
    return false;
  }
  ROS_DEBUG("Color blending filter opacity is = %f.", opacity_);

  if (!FilterBase < T > ::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Color blending filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Color blending filter output_layer = %s.", outputLayer_.c_str());
  return true;
}

template<typename T>
bool ColorBlendingFilter<T>::update(const T& mapIn, T& mapOut)
{
  const auto& background = mapIn[backgroundLayer_];
  const auto& foreground = mapIn[foregroundLayer_];

  mapOut = mapIn;
  mapOut.add(outputLayer_);
  auto& output = mapOut[outputLayer_];

  // For each cell in map.
  for (size_t i = 0; i < output.size(); ++i) {
    if (std::isnan(background(i))) {
      output(i) = foreground(i);
    } else if (std::isnan(foreground(i))) {
      output(i) = background(i);
    } else {
      Eigen::Array3f backgroundColor, foregroundColor, outputColor;
      Eigen::Vector3f color;
      colorValueToVector(background(i), color);
      backgroundColor = color.array();
      colorValueToVector(foreground(i), color);
      foregroundColor = color.array();

      switch (blendMode_) {
        case BlendModes::Normal:
          outputColor = (1.0 - opacity_) * backgroundColor + opacity_ * foregroundColor;
          break;
        case BlendModes::SoftLight:
          outputColor = (1.0 - opacity_) * backgroundColor
              + opacity_ * ((1.0 - 2.0 * backgroundColor) * foregroundColor.square() + 2.0 * foregroundColor * backgroundColor);
          break;
      }

      colorVectorToValue(Eigen::Vector3f(outputColor), output(i));
    }
  }

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::ColorBlendingFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
