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

namespace filters {

/*!
 * Weighted Sum Filter class to compute the weighted sum of different layers of a grid map.
 */
template<typename T>
class WeightedSumFilter : public FilterBase<T>
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

  //! List of types that are added together
  std::vector<std::string> additionTypes_;

  //! List of weights of the types that are added together
  std::vector<double> additionWeights_;

  //! map type for output of the summation.
  std::string typeOut_;

  //! Traversability map type.
  int normalize_;
};

} /* namespace */

#endif
