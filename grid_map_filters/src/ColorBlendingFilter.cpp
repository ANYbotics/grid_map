/*
 * ColorBlendingFilter.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/ColorBlendingFilter.hpp"

#include <math.h>

#include <Eigen/Dense>

#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

ColorBlendingFilter::ColorBlendingFilter() : opacity_(1.0), blendMode_(BlendModes::Normal) {}

ColorBlendingFilter::~ColorBlendingFilter() = default;

bool ColorBlendingFilter::configure() {
  if (!FilterBase::getParam(std::string("background_layer"), backgroundLayer_)) {
    ROS_ERROR("Color blending filter did not find parameter `background_layer`.");
    return false;
  }
  ROS_DEBUG("Color blending filter background layer is = %s.", backgroundLayer_.c_str());

  if (!FilterBase::getParam(std::string("foreground_layer"), foregroundLayer_)) {
    ROS_ERROR("Color blending filter did not find parameter `foreground_layer`.");
    return false;
  }
  ROS_DEBUG("Color blending filter foreground layer is = %s.", foregroundLayer_.c_str());

  std::string blendMode;
  if (!FilterBase::getParam(std::string("blend_mode"), blendMode)) {
    blendMode = "normal";
  }
  ROS_DEBUG("Color blending filter blend mode is = %s.", blendMode.c_str());
  if (blendMode == "normal") {
    blendMode_ = BlendModes::Normal;
  } else if (blendMode == "hard_light") {
    blendMode_ = BlendModes::HardLight;
  } else if (blendMode == "soft_light") {
    blendMode_ = BlendModes::SoftLight;
  } else {
    ROS_ERROR("Color blending filter blend mode `%s` does not exist.", blendMode.c_str());
    return false;
  }

  if (!FilterBase::getParam(std::string("opacity"), opacity_)) {
    ROS_ERROR("Color blending filter did not find parameter `opacity`.");
    return false;
  }
  ROS_DEBUG("Color blending filter opacity is = %f.", opacity_);

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Color blending filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Color blending filter output_layer = %s.", outputLayer_.c_str());
  return true;
}

bool ColorBlendingFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  const auto& background = mapIn[backgroundLayer_];
  const auto& foreground = mapIn[foregroundLayer_];

  mapOut = mapIn;
  mapOut.add(outputLayer_);
  auto& output = mapOut[outputLayer_];

  // For each cell in map.
  for (Eigen::Index i = 0; i < output.size(); ++i) {
    if (std::isnan(background(i))) {
      output(i) = foreground(i);
    } else if (std::isnan(foreground(i))) {
      output(i) = background(i);
    } else {
      Eigen::Array3f backgroundColor;
      Eigen::Array3f foregroundColor;
      Eigen::Array3f outputColor;
      Eigen::Vector3f color;
      colorValueToVector(background(i), color);
      backgroundColor = color.array();
      colorValueToVector(foreground(i), color);
      foregroundColor = color.array();

      switch (blendMode_) {
        case BlendModes::Normal:
          outputColor = (1.0 - opacity_) * backgroundColor + opacity_ * foregroundColor;
          break;
        case BlendModes::HardLight: {
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
        case BlendModes::SoftLight: {
          Eigen::Array3f blendedColor;
          // Photoshop.
          //          if (foregroundColor.mean() < 0.5) {
          //            blendedColor = 2.0 * backgroundColor * foregroundColor + backgroundColor.square() * (1.0 - 2.0 * foregroundColor);
          //          } else {
          //            blendedColor = 2.0 * backgroundColor * (1.0 - foregroundColor) + backgroundColor.sqrt() * (2.0 * foregroundColor
          //            - 1.0);
          //          }
          // Pegtop.
          blendedColor = ((1.0 - 2.0 * foregroundColor) * backgroundColor.square() + 2.0 * backgroundColor * foregroundColor);
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
