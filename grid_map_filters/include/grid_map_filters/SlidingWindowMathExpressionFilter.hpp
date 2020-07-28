/*
 * SlidingWindowMathExpressionFilter.hpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__SLIDINGWINDOWMATHEXPRESSIONFILTER_HPP_
#define GRID_MAP_FILTERS__SLIDINGWINDOWMATHEXPRESSIONFILTER_HPP_

#include <Eigen/Core>

#include <grid_map_core/grid_map_core.hpp>

#include <filters/filter_base.hpp>

#include <string>

#include "EigenLab/EigenLab.hpp"

namespace grid_map
{

/*!
 * Parse and evaluate a mathematical matrix expression within a sliding window on a layer of a grid map.
 */
template<typename T>
class SlidingWindowMathExpressionFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  SlidingWindowMathExpressionFilter();

  /*!
   * Destructor.
   */
  virtual ~SlidingWindowMathExpressionFilter();

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
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;

  //! EigenLab parser.
  EigenLab::Parser<Eigen::MatrixXf> parser_;

  //! Expression to parse.
  std::string expression_;

  //! Window size.
  int windowSize_;

  //! If window length (instead of window size) should be used.
  bool useWindowLength_;

  //! Window length.
  double windowLength_;

  //! If empty cells should be computed as well.
  bool isComputeEmptyCells_;

  //! Edge handling method.
  SlidingWindowIterator::EdgeHandling edgeHandling_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__SLIDINGWINDOWMATHEXPRESSIONFILTER_HPP_
