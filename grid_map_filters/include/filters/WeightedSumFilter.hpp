/*
 * WeightedSumFilter.hpp
 *
 *  Created on: Mar 16, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef WEIGHTEDSUMFILTER_HPP
#define WEIGHTEDSUMFILTER_HPP

#include <filters/filter_base.h>

#include <vector>
#include <string>

namespace grid_map_filters {

/*!
 * Weighted Sum Filter class to compute the weighted sum of different layers of a grid map.
 */
template<typename T>
class WeightedSumFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  WeightedSumFilter();

  /*!
   * Destructor.
   */
  virtual ~WeightedSumFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes the weighted sum of different layers of a grid map.
   * @param mapIn gridMap with the different layers to sum.
   * @param mapOut gridMap with an additional layer containing the sum.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! List of layers that are added together.
  std::vector<std::string> layers_;

  //! List of weights of the types that are added together
  std::vector<double> weights_;

  //! Map layer for output of the summation.
  std::string layerOut_;

  //! If true normalize weights.
  bool normalize_;
};

} /* namespace */

#endif
