/*
 * NormalColorMapFilter.cpp
 *
 *  Created on: Aug 22, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_filters/NormalColorMapFilter.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>

using namespace filters;

namespace grid_map {

template<typename T>
NormalColorMapFilter<T>::NormalColorMapFilter()
{
}

template<typename T>
NormalColorMapFilter<T>::~NormalColorMapFilter()
{
}

template<typename T>
bool NormalColorMapFilter<T>::configure()
{
  if (!FilterBase < T > ::getParam(std::string("input_layers_prefix"), inputLayersPrefix_)) {
    ROS_ERROR("Normal color map filter did not find parameter `input_layers_prefix`.");
    return false;
  }
  ROS_DEBUG("Normal color map filter input layers prefix is = %s.", inputLayersPrefix_.c_str());

  if (!FilterBase < T > ::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Normal color map filter did not find parameter `output_layer`.");
    return false;
  }
  ROS_DEBUG("Normal color map filter output_layer = %s.", outputLayer_.c_str());
  return true;
}

template<typename T>
bool NormalColorMapFilter<T>::update(const T& mapIn, T& mapOut)
{
  const auto& normalX = mapIn[inputLayersPrefix_ + "x"];
  const auto& normalY = mapIn[inputLayersPrefix_ + "y"];
  const auto& normalZ = mapIn[inputLayersPrefix_ + "z"];

  mapOut = mapIn;
  mapOut.add(outputLayer_);
  auto& color = mapOut[outputLayer_];

  // X: -1 to +1 : Red: 0 to 255
  // Y: -1 to +1 : Green: 0 to 255
  // Z:  0 to  1 : Blue: 128 to 255

  // For each cell in map.
  for (size_t i = 0; i < color.size(); ++i) {
    const Eigen::Vector3f colorVector((normalX(i) + 1.0) / 2.0,
                                      (normalY(i) + 1.0) / 2.0,
                                      (normalZ(i) / 2.0) + 0.5);
    colorVectorToValue(colorVector, color(i));
  }

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::NormalColorMapFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
