/*
 * MedianFillFilter.hpp
 *
 *  Created on: September 7, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */


#pragma once

#include <filters/filter_base.h>

#include <string>

namespace grid_map {

/*!
 * Uses Boost accumulators to fill holes in the input layer by the median of the surrounding values.
 */
template<typename T>
class MedianFillFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  MedianFillFilter();

  /*!
   * Destructor.
   */
  virtual ~MedianFillFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Adds a new output layer to the map.
   * Uses the Boost accumulator median in the input layer.
   * Saves the filter output in mapOut[output_layer].
   * @param mapIn grid map containing input layer
   * @param mapOut grid map containing mapIn and median filtered input layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Median filtering radius of NaN values in the input.
  double fillHoleRadius_;

  //! Median filtering radius for existing values in the input.
  double existingValueRadius_;

  //! Flag indicating whether to also filter finite values.
  bool filterExistingValues_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

} /* namespace */
