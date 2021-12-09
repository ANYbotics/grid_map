/*
 * MathExpressionFilter.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MathExpressionFilter.hpp"

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

  // TODO Can we make caching work with changing shared variable?
//  parser_.setCacheExpressions(true);
  return true;
}

template<typename T>
bool MathExpressionFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  for (const auto& layer : mapOut.getLayers()) {
    parser_.var(layer).setShared(mapOut[layer]);
  }
  EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
  mapOut.add(outputLayer_, result.matrix());
  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::MathExpressionFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
