/*
 * MathExpressionFilter.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <string>
#include "EigenLab/EigenLab.h"

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

/*!
 * Parses and evaluates a mathematical matrix expression with layers of a grid map.
 */
class MathExpressionFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  MathExpressionFilter();

  /*!
   * Destructor.
   */
  ~MathExpressionFilter() override;

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Takes the minimum out of different layers of a grid map.
   * @param mapIn gridMap with the different layers to take the min.
   * @param mapOut gridMap with an additional layer containing the sum.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! EigenLab parser.
  EigenLab::Parser<Eigen::MatrixXf> parser_;

  //! Expression to parse.
  std::string expression_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
