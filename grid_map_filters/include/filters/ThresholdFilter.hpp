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

namespace grid_map_filters {

/*!
 * Threshold Filter class that sets values below a lower threshold to 0 and
 * values above an upper threshold to 1.
 */
template<typename T>
class ThresholdFilter : public filters::FilterBase<T>
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
   * Uses either an upper or lower threshold. If the threshold is exceeded
   * the cell value is set to the predefined value setTo_.
   * @param mapIn GridMap with the different layers to apply a threshold.
   * @param mapOut GridMap with the threshold applied to the layers.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! List of layers the threshold should be applied
  std::vector<std::string> layers_;

  //! Lower Threshold
  double lowerThreshold_;

  //! Upper Threshold
  double upperThreshold_;

  //! If threshold triggered set to this value
  double setTo_;

  //! Booleans to decide which threshold should be used.
  bool useLowerThreshold_, useUpperThreshold_;
};

} /* namespace */

#endif
