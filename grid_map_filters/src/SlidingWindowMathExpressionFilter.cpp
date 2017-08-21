/*
 * SlidingWindowMathExpressionFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_filters/SlidingWindowMathExpressionFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

template<typename T>
SlidingWindowMathExpressionFilter<T>::SlidingWindowMathExpressionFilter()
: windowSize_(3),
  useWindowLength_(false),
  windowLength_(0.0),
  isComputeEmptyCells_(true)
{
}

template<typename T>
SlidingWindowMathExpressionFilter<T>::~SlidingWindowMathExpressionFilter()
{
}

template<typename T>
bool SlidingWindowMathExpressionFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("layer"), layer_)) {
    ROS_ERROR("SlidingWindowMathExpressionFilter did not find parameter 'layer'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("expression"), expression_)) {
    ROS_ERROR("SlidingWindowMathExpressionFilter did not find parameter 'expression'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("window_size"), windowSize_)) {
    if (FilterBase<T>::getParam(std::string("window_length"), windowLength_)) {
      useWindowLength_ = true;
    }
  }

  if (!FilterBase<T>::getParam(std::string("compute_empty_cells"), isComputeEmptyCells_)) {
    ROS_ERROR("SlidingWindowMathExpressionFilter did not find parameter 'compute_empty_cells'.");
    return false;
  }

  parser_.setCacheExpressions(true);
  return true;
}

template<typename T>
bool SlidingWindowMathExpressionFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  Matrix& outputData = mapOut[layer_];
  for (grid_map::SlidingWindowIterator iterator(mapIn, layer_, windowSize_); !iterator.isPastEnd(); ++iterator) {
    parser_.var(layer_).setLocal(iterator.getData());
    EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
    if (result.matrix().cols() == 1 && result.matrix().rows() == 1) {
      outputData(iterator.getLinearIndex()) = result.matrix()(0);
    } else {
      ROS_ERROR("SlidingWindowMathExpressionFilter could not apply filter because expression has to result in a scalar!");
    }
  }
  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(MinFilter, grid_map::SlidingWindowMathExpressionFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
