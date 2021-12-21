/*
 * MathExpressionFilter.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MathExpressionFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

MathExpressionFilter::MathExpressionFilter() = default;

MathExpressionFilter::~MathExpressionFilter() = default;

bool MathExpressionFilter::configure() {
  if (!FilterBase::getParam(std::string("expression"), expression_)) {
    ROS_ERROR("MathExpressionFilter did not find parameter 'expression'.");
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("MathExpressionFilter did not find parameter 'output_layer'.");
    return false;
  }

  // TODO Can we make caching work with changing shared variable?
  //  parser_.setCacheExpressions(true);
  return true;
}

bool MathExpressionFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  for (const auto& layer : mapOut.getLayers()) {
    parser_.var(layer).setShared(mapOut[layer]);
  }
  EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
  mapOut.add(outputLayer_, result.matrix());
  return true;
}

}  // namespace grid_map
