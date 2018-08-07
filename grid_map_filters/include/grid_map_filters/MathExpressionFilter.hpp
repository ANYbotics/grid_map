/*
 * MathExpressionFilter.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "EigenLab/EigenLab.h"

#include <filters/filter_base.h>

#include <string>

namespace grid_map {

/*!
 * Parses and evaluates a mathematical matrix expression with layers of a grid map.
 */
template<typename T>
class MathExpressionFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  MathExpressionFilter();

  /*!
   * Destructor.
   */
  virtual ~MathExpressionFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Takes the minimum out of different layers of a grid map.
   * @param mapIn gridMap with the different layers to take the min.
   * @param mapOut gridMap with an additional layer containing the sum.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! EigenLab parser.
  EigenLab::Parser<Eigen::MatrixXf> parser_;

  //! Expression to parse.
  std::string expression_;

  //! Output layer name.
  std::string outputLayer_;
};

} /* namespace */
