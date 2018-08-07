/*
 * ThresholdFilter.hpp
 *
 *  Created on: Mar 16, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>

#include <string>
#include <vector>

namespace grid_map {

/*!
 * Threshold filter class to set values below/above a threshold to a
 * specified value.
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
   * Configures the filter from parameters on the parameter server.
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

  //! Layer the threshold should be applied to.
  std::string layer_;

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
