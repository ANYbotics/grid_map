/*
 * CurvatureFilter.cpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_filters/CurvatureFilter.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <Eigen/Dense>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
CurvatureFilter<T>::CurvatureFilter()
{
}

template<typename T>
CurvatureFilter<T>::~CurvatureFilter()
{
}

template<typename T>
bool CurvatureFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("input_layer"), inputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Curvature filter did not find parameter `input_layer`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Curvature filter input layer is = %s.",
    inputLayer_.c_str());

  if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Curvature filter did not find parameter `output_layer`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Curvature filter output_layer = %s.",
    outputLayer_.c_str());

  return true;
}

template<typename T>
bool CurvatureFilter<T>::update(const T & mapIn, T & mapOut)
{
  if (!mapIn.isDefaultStartIndex()) {
    throw std::runtime_error(
            "CurvatureFilter cannot be used with grid maps"
            " that don't have a default buffer start index.");
  }

  mapOut = mapIn;
  mapOut.add(outputLayer_);
  auto & input = mapOut[inputLayer_];
  auto & curvature = mapOut[outputLayer_];
  const float L2 = mapOut.getResolution() * mapOut.getResolution();

  for (int64_t j = 0; j < input.cols(); ++j) {
    for (int64_t i = 0; i < input.rows(); ++i) {
      // http://help.arcgis.com/en/arcgisdesktop/10.0/help/index.html#/How_Curvature_works/00q90000000t000000/
      if (!std::isfinite(input(i, j))) {continue;}
      float D =
        ((input(
          i,
          j ==
          0 ? j : j - 1) + input(i, j == input.cols() - 1 ? j : j + 1)) / 2.0 - input(i, j)) / L2;
      float E =
        ((input(
          i == 0 ? i : i - 1,
          j) + input(i == input.rows() - 1 ? i : i + 1, j)) / 2.0 - input(i, j)) / L2;
      if (!std::isfinite(D)) {D = 0.0;}
      if (!std::isfinite(E)) {E = 0.0;}
      curvature(i, j) = -2.0 * (D + E);
    }
  }

  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::CurvatureFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
