/*
 * ThresholdFilter.hpp
 *
 *  Created on: Mar 16, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef THRESHOLDFILTER_HPP
#define THRESHOLDFILTER_HPP

#include <filters/filter_base.h>

#include <vector>
#include <string>

namespace filters {

/*!
 * Threshold Filter class that sets values below a lower threshold to 0 and
 * values above an upper threshold to 1.
 */
template<typename T>
class ThresholdFilter : public FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  ThresholdFilter();

  /*!
   * Destructor.
   */
  virtual ~ThresholdFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Sets values below a lower threshold to 0  an values above an upper
   * threshold to 1
   * @param mapIn gridMap with the different layers to apply a threshold.
   * @param mapOut gridMap with the threshold applied to the layers.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! List of layers the threshold should be applied
  std::vector<std::string> thresholdTypes_;

  //! Lower Threshold
  double lowerThreshold_;

  //! Upper Threshold
  double upperThreshold_;
};

} /* namespace */

#endif
