/*
 * MedianFillFilter.hpp
 *
 *  Created on: September 7, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>

#include <Eigen/Core>
#include <string>

#include <grid_map_core/TypeDefs.hpp>

namespace grid_map {

/*!
 * Uses Boost accumulators to fill holes in the input layer by the median of the surrounding values.
 */
template <typename T>
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

 protected:
  /*!
   * Returns the median of the values in inputData in the neighbourhood around the centerIndex. The size of the quadratic neighbourhood is
   * specified by radiusInPixels. If the number of values is even the "lower center" value is taken, eg with four values the second lowest
   * is taken as median.
   * @param inputMap The data layer to compute a local median.
   * @param centerIndex The center cell of the neighbourhood.
   * @param radiusInPixels The maximum L_inf distance from index.
   * @param bufferSize The buffer size of the input
   * @return The median of finites in the specified neighbourhood.
   */
  float getMedian(Eigen::Ref<const grid_map::Matrix> inputMap, const grid_map::Index& centerIndex, const size_t radiusInPixels,
                  const grid_map::Size bufferSize);

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

}  // namespace grid_map
