/*
 * MathExpressionFilter.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_filters/MathExpressionFilter.hpp"
#include "EigenLab/EigenLab.h"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

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
  if (!FilterBase<T>::getParam(std::string("expression"), expression_)) {
    ROS_ERROR("MathExpressionFilter did not find parameter 'expression'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("MathExpressionFilter did not find parameter 'output_layer'.");
    return false;
  }

  return true;
}

template<typename T>
bool MathExpressionFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  EigenLab::Parser<Eigen::MatrixXf> parser;

  for (const auto& layer : mapOut.getLayers()) {
    parser.var(layer).setLocal(mapOut[layer]);
  }

  EigenLab::Value<Eigen::MatrixXf> result;
  result = parser.eval(expression_);
  mapOut.add(outputLayer_, result.matrix());
  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(MinFilter, grid_map::MathExpressionFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
