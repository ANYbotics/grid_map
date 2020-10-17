/*
 * ColorBlendingFilter.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_filters/ColorBlendingFilter.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <Eigen/Dense>
#include <math.h>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
ColorBlendingFilter<T>::ColorBlendingFilter()
: opacity_(1.0),
  blendMode_(BlendModes::Normal)
{
}

template<typename T>
ColorBlendingFilter<T>::~ColorBlendingFilter()
{
}

template<typename T>
bool ColorBlendingFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("background_layer"), backgroundLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Color blending filter did not find parameter `background_layer`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Color blending filter background layer is = %s.",
    backgroundLayer_.c_str());

  if (!param_reader.get(std::string("foreground_layer"), foregroundLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Color blending filter did not find parameter `foreground_layer`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Color blending filter foreground layer is = %s.",
    foregroundLayer_.c_str());

  std::string blendMode;
  if (!param_reader.get(std::string("blend_mode"), blendMode)) {
    blendMode = "normal";
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Color blending filter blend mode is = %s.",
    blendMode.c_str());
  if (blendMode == "normal") {
    blendMode_ = BlendModes::Normal;
  } else if (blendMode == "hard_light") {
    blendMode_ = BlendModes::HardLight;
  } else if (blendMode == "soft_light") {blendMode_ = BlendModes::SoftLight;} else {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Color blending filter blend mode `%s` does not exist.", blendMode.c_str());
    return false;
  }

  if (!param_reader.get(std::string("opacity"), opacity_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Color blending filter did not find parameter `opacity`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Color blending filter opacity is = %f.", opacity_);

  if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Color blending filter did not find parameter `output_layer`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Color blending filter output_layer = %s.",
    outputLayer_.c_str());
  return true;
}

template<typename T>
bool ColorBlendingFilter<T>::update(const T & mapIn, T & mapOut)
{
  const auto & background = mapIn[backgroundLayer_];
  const auto & foreground = mapIn[foregroundLayer_];

  mapOut = mapIn;
  mapOut.add(outputLayer_);
  auto & output = mapOut[outputLayer_];

  // For each cell in map.
  for (int64_t i = 0; i < output.size(); ++i) {
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
        case BlendModes::HardLight:
          {
            Eigen::Array3f blendedColor;
            if (foregroundColor.mean() < 0.5) {
              blendedColor = 2.0 * backgroundColor * foregroundColor;
            } else {
              blendedColor = 1.0 - 2.0 * (1.0 - backgroundColor) * (1.0 - foregroundColor);
            }
            if (opacity_ == 1.0) {
              outputColor = blendedColor;
            } else {
              outputColor = (1.0 - opacity_) * backgroundColor + opacity_ * blendedColor;
            }

            break;
          }
        case BlendModes::SoftLight:
          {
            Eigen::Array3f blendedColor;
            // Photoshop.
//          if (foregroundColor.mean() < 0.5) {
//            blendedColor = 2.0 * backgroundColor * foregroundColor +
//              backgroundColor.square() * (1.0 - 2.0 * foregroundColor);
//          } else {
//            blendedColor = 2.0 * backgroundColor * (1.0 - foregroundColor) +
//              backgroundColor.sqrt() * (2.0 * foregroundColor - 1.0);
//          }
            // Pegtop.
            blendedColor =
              ((1.0 - 2.0 * foregroundColor) * backgroundColor.square() + 2.0 * backgroundColor *
              foregroundColor);
            if (opacity_ == 1.0) {
              outputColor = blendedColor;
            } else {
              outputColor = (1.0 - opacity_) * backgroundColor + opacity_ * blendedColor;
            }

            break;
          }
      }

      colorVectorToValue(Eigen::Vector3f(outputColor), output(i));
    }
  }

  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::ColorBlendingFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
