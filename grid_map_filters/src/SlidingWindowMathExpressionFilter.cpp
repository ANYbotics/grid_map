/*
 * SlidingWindowMathExpressionFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/SlidingWindowMathExpressionFilter.hpp"

#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

template<typename T>
SlidingWindowMathExpressionFilter<T>::SlidingWindowMathExpressionFilter()
: windowSize_(3),
  useWindowLength_(false),
  windowLength_(0.0),
  isComputeEmptyCells_(true),
  edgeHandling_(SlidingWindowIterator::EdgeHandling::INSIDE)
{
}

template<typename T>
SlidingWindowMathExpressionFilter<T>::~SlidingWindowMathExpressionFilter()
{
}

template<typename T>
bool SlidingWindowMathExpressionFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("SlidingWindowMathExpressionFilter did not find parameter 'input_layer'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("SlidingWindowMathExpressionFilter did not find parameter 'output_layer'.");
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

  std::string edgeHandlingMethod;
  if (!FilterBase<T>::getParam(std::string("edge_handling"), edgeHandlingMethod)) {
    ROS_ERROR("SlidingWindowMathExpressionFilter did not find parameter 'edge_handling'.");
    return false;
  }
  if (edgeHandlingMethod == "inside") edgeHandling_ = SlidingWindowIterator::EdgeHandling::INSIDE;
  else if (edgeHandlingMethod == "crop") edgeHandling_ = SlidingWindowIterator::EdgeHandling::CROP;
  else if (edgeHandlingMethod == "empty") edgeHandling_ = SlidingWindowIterator::EdgeHandling::EMPTY;
  else if (edgeHandlingMethod == "mean") edgeHandling_ = SlidingWindowIterator::EdgeHandling::MEAN;
  else {
    ROS_ERROR("SlidingWindowMathExpressionFilter did not find method '%s' for edge handling.", edgeHandlingMethod.c_str());
    return false;
  }

  // TODO Can we make caching work with changing shared variable?
//  parser_.setCacheExpressions(true);
  return true;
}

template<typename T>
bool SlidingWindowMathExpressionFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(outputLayer_);
  Matrix& outputData = mapOut[outputLayer_];
  grid_map::SlidingWindowIterator iterator(mapIn, inputLayer_, edgeHandling_, windowSize_);
  if (useWindowLength_) iterator.setWindowLength(mapIn, windowLength_);
  for (; !iterator.isPastEnd(); ++iterator) {
    parser_.var(inputLayer_).setLocal(iterator.getData());
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

PLUGINLIB_EXPORT_CLASS(grid_map::SlidingWindowMathExpressionFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
