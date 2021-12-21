/*
 * ColorMapFilter.cpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/ColorMapFilter.hpp"

#include <Eigen/Dense>

#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

ColorMapFilter::ColorMapFilter() : min_(0.0), max_(1.0) {}

ColorMapFilter::~ColorMapFilter() = default;

bool ColorMapFilter::configure() {
  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Color map filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("Color map filter input_layer = %s.", inputLayer_.c_str());

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Color map filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Color map filter output_layer = %s.", outputLayer_.c_str());

  if (!FilterBase::getParam(std::string("min/value"), min_)) {
    ROS_ERROR("Color map filter did not find parameter `min/value`.");
    return false;
  }
  if (!FilterBase::getParam(std::string("max/value"), max_)) {
    ROS_ERROR("Color map filter did not find parameter `max/value`.");
    return false;
  }

  std::vector<double> minColor;
  if (!FilterBase::getParam(std::string("min/color"), minColor)) {
    ROS_ERROR("Color map filter did not find parameter `min/color`.");
    return false;
  }
  if (minColor.size() != 3) {
    ROS_ERROR("Color map filter parameter `min/color` needs to be of size 3.");
    return false;
  }
  minColor_ << minColor[0], minColor[1], minColor[2];

  std::vector<double> maxColor;
  if (!FilterBase::getParam(std::string("max/color"), maxColor)) {
    ROS_ERROR("Color map filter did not find parameter `max/color`.");
    return false;
  }
  if (maxColor.size() != 3) {
    ROS_ERROR("Color map filter parameter `max/color` needs to be of size 3.");
    return false;
  }
  maxColor_ << maxColor[0], maxColor[1], maxColor[2];

  return true;
}

bool ColorMapFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  const auto& input = mapIn[inputLayer_];
  mapOut.add(outputLayer_);
  auto& output = mapOut[outputLayer_];

  const double range = max_ - min_;
  const Eigen::Vector3f colorRange = maxColor_ - minColor_;

  // For each cell in map.
  for (Eigen::Index i = 0; i < output.size(); ++i) {
    if (!std::isfinite(input(i))) {
      continue;
    }
    const double value = std::min<float>(std::max<float>(input(i), min_), max_);
    const double factor = (value - min_) / range;
    const Eigen::Vector3f color = minColor_ + factor * colorRange;
    colorVectorToValue(color, output(i));
  }

  return true;
}

}  // namespace grid_map
