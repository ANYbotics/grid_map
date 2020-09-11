/*
 * MedianFillFilter.cpp
 *
 *  Created on: September 7, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MedianFillFilter.hpp"

// Wrap as ROS Filter
#include <pluginlib/class_list_macros.h>

// Median filtering with boost accumulators
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map {

template <typename T>
MedianFillFilter<T>::MedianFillFilter() : fillHoleRadius_(0.05), filterExistingValues_(false) {}

template <typename T>
MedianFillFilter<T>::~MedianFillFilter() = default;

template <typename T>
bool MedianFillFilter<T>::configure() {
  if (!FilterBase<T>::getParam(std::string("fill_hole_radius"), fillHoleRadius_)) {
    ROS_ERROR("Median filter did not find parameter fill_hole_radius.");
    return false;
  }

  if (fillHoleRadius_ < 0.0) {
    ROS_ERROR("fill_hole_radius must be greater than zero.");
    return false;
  }

  ROS_DEBUG("fill_hole_radius = %f.", fillHoleRadius_);

  if (!FilterBase<T>::getParam(std::string("filter_existing_values"), filterExistingValues_)) {
    ROS_INFO("Median filter did not find parameter filter_existing_values. Not filtering existing values.");
    filterExistingValues_ = false;
  }

  ROS_DEBUG("filter_existing_values = %b.", filterExistingValues_);

  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Median filter did not find parameter `input_layer`.");
    return false;
  }

  if (filterExistingValues_) {
    if (!FilterBase<T>::getParam(std::string("existing_value_radius"), existingValueRadius_)) {
      ROS_ERROR("Median filter did not find parameter existing_value_radius.");
      return false;
    }

    if (existingValueRadius_ < 0.0) {
      ROS_ERROR("existing_value_radius must be greater than zero.");
      return false;
    }

    ROS_DEBUG("existing_value_radius = %f.", existingValueRadius_);
  }

  ROS_DEBUG("Median input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Median filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("Median output layer = %s.", outputLayer_.c_str());

  return true;
}

template <typename T>
bool MedianFillFilter<T>::update(const T& mapIn, T& mapOut) {
  // Type definitions for readability.
  using boost::accumulators::accumulator_set;
  using boost::accumulators::count;
  using boost::accumulators::features;
  using boost::accumulators::median;
  using median_tag = boost::accumulators::tag::median;
  using count_tag = boost::accumulators::tag::count;
  using MedianAccumulatorT = accumulator_set<double, features<median_tag, count_tag> >;

  // Copy input map and add new layer to it.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  const auto radiusInPixels = static_cast<size_t>(fillHoleRadius_ / mapIn.getResolution());
  const auto existingValueRadiusInPixels = static_cast<size_t>(existingValueRadius_ / mapIn.getResolution());
  const auto kernelLength = radiusInPixels * 2 + 1;
  const grid_map::Index& bufferSize = mapOut.getSize();

  grid_map::Matrix& inputData = mapOut[inputLayer_];
  grid_map::Matrix& outputData = mapOut[outputLayer_];
  grid_map::Matrix& varianceData = mapOut["variance"];

  // Define some local functions.
  /*
   * @brief Returns the neighbourhood iterator of an index given the neighbourhood radius.
   * It checks the boundaries of the submap to compute the rectangular neighbourhood patch to iterate over.
   * @param index The center index of which we want the neighbourhood iterator.
   * @param radiusInPixels The number of pixels go in x / y direction from index to still be in the neighbourhood. I.e: ||neighbour -
   * index||_{inf} <= radiusInPixels.
   * @return A submap iterator of mapOut that iterates over all neighbours of index, including itself.
   */
  auto getNeighbourhoodIterator = [&bufferSize, &mapOut](const grid_map::Index& index, int radiusInPixels) {
    // Bound the median window to the GridMap boundaries. Calculate the neighbour patch.
    grid_map::Index topLeftIndex = index - grid_map::Index(radiusInPixels, radiusInPixels);
    grid_map::Index bottomRightIndex = index + grid_map::Index(radiusInPixels, radiusInPixels);
    grid_map::boundIndexToRange(topLeftIndex, bufferSize);
    grid_map::boundIndexToRange(bottomRightIndex, bufferSize);
    grid_map::Index neighbourPatchSize = bottomRightIndex - topLeftIndex + grid_map::Index{1, 1};

    // Iterate over the local neighbourhood.
    return grid_map::SubmapIterator(mapOut, topLeftIndex, neighbourPatchSize);
  };

  auto applyAccumulatorToFinites = [](const grid_map::Matrix& dataLayer, grid_map::SubmapIterator neighbourIterator,
                                      MedianAccumulatorT& accumulator) {
    // Iterate over the local neighbourhood.

    for (; !neighbourIterator.isPastEnd(); ++neighbourIterator) {
      const Index neighbourIndex(*neighbourIterator);
      const auto& neighbourValue = dataLayer(neighbourIndex(0), neighbourIndex(1));
      if (std::isfinite(neighbourValue)) {  // Calculate the median of the finite neighbours.
        accumulator(neighbourValue);
      }
    }
  };

  // Iterate through the entire GridMap and update NaN values with the median.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    const auto& inputValue = inputData(index(0), index(1));
    auto& outputValue = outputData(index(0), index(1));
    auto& variance = varianceData(index(0), index(1));

    if (not std::isfinite(inputValue)) {  // Fill the NaN input value with the median.
      MedianAccumulatorT medianAccumulator;
      auto neighbourIterator = getNeighbourhoodIterator(index, radiusInPixels);
      applyAccumulatorToFinites(inputData, neighbourIterator, medianAccumulator);

      if (count(medianAccumulator) >= 1.0f) {  // If we have at least one valid input value in the neighbourhood we assign the median.
        outputValue = median(medianAccumulator);
        variance = 0.1;
      }

    } else if (filterExistingValues_) {  // Value is already finite. Optionally add some filtering.
      MedianAccumulatorT medianAccumulator;
      auto neighbourIterator = getNeighbourhoodIterator(index, existingValueRadiusInPixels);
      applyAccumulatorToFinites(inputData, neighbourIterator, medianAccumulator);

      if (count(medianAccumulator) >= 1.0f) {
        outputValue = median(medianAccumulator);
      }

    } else {  // Dont do any filtering, just take the input value.
      outputValue = inputValue;
    }
  }

  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::MedianFillFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::MedianFillFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
