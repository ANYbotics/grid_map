/*
 * MathExpressionFilter.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MathExpressionFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
MathExpressionFilter<T>::MathExpressionFilter()
{
}

template<typename T>
MathExpressionFilter<T>::~MathExpressionFilter()
{
}

template<typename T>
bool MathExpressionFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("expression"), expression_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "MathExpressionFilter did not find parameter 'expression'.");
    return false;
  }

  if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "MathExpressionFilter did not find parameter 'output_layer'.");
    return false;
  }

  // TODO(needs_assignment): Can we make caching work with changing shared variable?
//  parser_.setCacheExpressions(true);
  return true;
}

template<typename T>
bool MathExpressionFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;
  for (const auto & layer : mapOut.getLayers()) {
    parser_.var(layer).setShared(mapOut[layer]);
  }
  EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
  mapOut.add(outputLayer_, result.matrix());
  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::MathExpressionFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
