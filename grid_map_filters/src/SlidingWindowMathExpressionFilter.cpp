/*
 * SlidingWindowMathExpressionFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/SlidingWindowMathExpressionFilter.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

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
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get(std::string("input_layer"), inputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "SlidingWindowMathExpressionFilter did not find parameter 'input_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "SlidingWindowMathExpressionFilter did not find parameter 'output_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("expression"), expression_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "SlidingWindowMathExpressionFilter did not find parameter 'expression'.");
    return false;
  }

  if (!param_reader.get(std::string("window_size"), windowSize_)) {
    if (param_reader.get(std::string("window_length"), windowLength_)) {
      useWindowLength_ = true;
    }
  }

  if (!param_reader.get(std::string("compute_empty_cells"), isComputeEmptyCells_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "SlidingWindowMathExpressionFilter did not find parameter 'compute_empty_cells'.");
    return false;
  }

  std::string edgeHandlingMethod;
  if (!param_reader.get(std::string("edge_handling"), edgeHandlingMethod)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "SlidingWindowMathExpressionFilter did not find parameter 'edge_handling'.");
    return false;
  }
  if (edgeHandlingMethod == "inside") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::INSIDE;
  } else if (edgeHandlingMethod == "crop") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::CROP;
  } else if (edgeHandlingMethod == "empty") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::EMPTY;
  } else if (edgeHandlingMethod == "mean") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::MEAN;
  } else {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "SlidingWindowMathExpressionFilter did not find method '%s' for edge handling.",
      edgeHandlingMethod.c_str());
    return false;
  }

  // TODO(needs_assignment): Can we make caching work with changing shared variable?
//  parser_.setCacheExpressions(true);
  return true;
}

template<typename T>
bool SlidingWindowMathExpressionFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;
  mapOut.add(outputLayer_);
  Matrix & outputData = mapOut[outputLayer_];
  grid_map::SlidingWindowIterator iterator(mapIn, inputLayer_, edgeHandling_, windowSize_);
  if (useWindowLength_) {iterator.setWindowLength(mapIn, windowLength_);}
  for (; !iterator.isPastEnd(); ++iterator) {
    parser_.var(inputLayer_).setLocal(iterator.getData());
    EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
    if (result.matrix().cols() == 1 && result.matrix().rows() == 1) {
      outputData(iterator.getLinearIndex()) = result.matrix()(0);
    } else {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "SlidingWindowMathExpressionFilter could not apply filter"
        " because expression has to result in a scalar!");
    }
  }
  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::SlidingWindowMathExpressionFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
