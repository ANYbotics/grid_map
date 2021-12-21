/*
 * LightIntensityFilter.cpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/LightIntensityFilter.hpp"

#include <Eigen/Dense>

#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

LightIntensityFilter::LightIntensityFilter() = default;

LightIntensityFilter::~LightIntensityFilter() = default;

bool LightIntensityFilter::configure() {
  if (!FilterBase::getParam(std::string("input_layers_prefix"), inputLayersPrefix_)) {
    ROS_ERROR("Light intensity filter did not find parameter `input_layers_prefix`.");
    return false;
  }
  ROS_DEBUG("Light intensity filter input layers prefix is = %s.", inputLayersPrefix_.c_str());

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Light intensity filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Light intensity filter output_layer = %s.", outputLayer_.c_str());

  std::vector<double> lightDirection;
  if (!FilterBase::getParam(std::string("light_direction"), lightDirection)) {
    ROS_ERROR("Light intensity filter did not find parameter `light_direction`.");
    return false;
  }
  if (lightDirection.size() != 3) {
    ROS_ERROR("Light intensity filter parameter `light_direction` needs to be of size 3.");
    return false;
  }
  lightDirection_ << lightDirection[0], lightDirection[1], lightDirection[2];
  lightDirection_.normalize();
  ROS_DEBUG_STREAM("Light intensity filter light_direction: " << lightDirection_.transpose() << ".");

  return true;
}

bool LightIntensityFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  const auto& normalX = mapIn[inputLayersPrefix_ + "x"];
  const auto& normalY = mapIn[inputLayersPrefix_ + "y"];
  const auto& normalZ = mapIn[inputLayersPrefix_ + "z"];

  mapOut = mapIn;
  mapOut.add(outputLayer_);
  auto& color = mapOut[outputLayer_];

  // For each cell in map.
  for (Eigen::Index i = 0; i < color.size(); ++i) {
    if (!std::isfinite(normalZ(i))) {
      color(i) = NAN;
      continue;
    }
    const Eigen::Vector3f normal(normalX(i), normalY(i), normalZ(i));
    const float intensity = std::max<float>(-normal.dot(lightDirection_), 0.0);
    const Eigen::Vector3f colorVector = Eigen::Vector3f::Ones() * intensity;
    colorVectorToValue(colorVector, color(i));
  }

  return true;
}

}  // namespace grid_map
