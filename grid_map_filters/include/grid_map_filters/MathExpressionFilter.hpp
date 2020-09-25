/*
 * MathExpressionFilter.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__MATHEXPRESSIONFILTER_HPP_
#define GRID_MAP_FILTERS__MATHEXPRESSIONFILTER_HPP_

#include <filters/filter_base.hpp>

#include <string>

#include "EigenLab/EigenLab.hpp"

namespace grid_map
{

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
  bool configure() override;

  /*!
   * Takes the minimum out of different layers of a grid map.
   * @param mapIn gridMap with the different layers to take the min.
   * @param mapOut gridMap with an additional layer containing the sum.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! EigenLab parser.
  EigenLab::Parser<Eigen::MatrixXf> parser_;

  //! Expression to parse.
  std::string expression_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__MATHEXPRESSIONFILTER_HPP_
