/*
 * CVaRFilter.hpp
 *
 *  Created on: Nov 28, 2025
 *      Author: Riana Gagnon (based on GridMap filters by ANYbotics)
 */
#ifndef GRID_MAP_FILTERS__CVARFILTER_HPP_
#define GRID_MAP_FILTERS__CVARFILTER_HPP_
#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <Eigen/Core>
#include <string>
#include <vector>

namespace grid_map
{
/*!
 * Computes the Conditional Value-at-Risk (CVaR) of values inside a sliding window
 * on a given input layer, and writes the result to a new output layer.
 */
template<typename T>
class CVaRFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor.
   */
  CVaRFilter();

  /*!
   * Destructor.
   */
  virtual ~CVaRFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Applies the CVaR calculation over a sliding window.
   * @param mapIn Input grid map.
   * @param mapOut Output grid map with added CVaR layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;

  //! CVaR confidence level α (e.g., 0.2 means worst 20% tail).
  double alpha_;

  //! Sliding window size (number of cells).
  int windowSize_;

  //! If window length (meters) should be used instead of window size.
  bool useWindowLength_;

  //! Sliding window length (meters).
  double windowLength_;

  //! Whether to compute values even for empty cells.
  bool isComputeEmptyCells_;

  //! Edge handling method (INSIDE, CROP, EMPTY, MEAN).
  SlidingWindowIterator::EdgeHandling edgeHandling_;
};

}  // namespace grid_map

#endif  // GRID_MAP_FILTERS__CVARFILTER_HPP_