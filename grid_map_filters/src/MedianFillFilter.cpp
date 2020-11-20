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

  ROS_DEBUG("Filter_existing_values = %s.", filterExistingValues_ ? "true" : "false");

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
float MedianFillFilter<T>::getMedian(Eigen::Ref<const grid_map::Matrix> inputMap, const grid_map::Index& centerIndex,
                                     const size_t radiusInPixels, const grid_map::Size bufferSize) {
  // Bound the median window to the GridMap boundaries. Calculate the neighbour patch.
  grid_map::Index topLeftIndex = centerIndex - grid_map::Index(radiusInPixels, radiusInPixels);
  grid_map::Index bottomRightIndex = centerIndex + grid_map::Index(radiusInPixels, radiusInPixels);
  grid_map::boundIndexToRange(topLeftIndex, bufferSize);
  grid_map::boundIndexToRange(bottomRightIndex, bufferSize);
  const grid_map::Index neighbourPatchSize = bottomRightIndex - topLeftIndex + grid_map::Index{1, 1};

  // Extract local neighbourhood.
  const auto& neighbourhood = inputMap.block(topLeftIndex(0), topLeftIndex(1), neighbourPatchSize(0), neighbourPatchSize(1));

  const size_t cols = neighbourhood.cols();

  std::vector<float> cellValues;
  cellValues.reserve(neighbourhood.rows() * neighbourhood.cols());

  for (size_t row = 0; row < neighbourhood.rows(); ++row) {
    const auto& currentRow = neighbourhood.row(row);
    for (size_t col = 0; col < cols; ++col) {
      const float& cellValue = currentRow[col];
      if (std::isfinite(cellValue)) {  // Calculate the median of the finite neighbours.
        cellValues.emplace_back(cellValue);
      }
    }
  }

  if (cellValues.empty()) {
    return static_cast<float>(NAN);
  } else {  // Compute the median of the finite values in the neighbourhood.
    std::nth_element(cellValues.begin(), cellValues.begin() + cellValues.size() / 2, cellValues.end());
    return cellValues[cellValues.size() / 2];
  }
}

template <typename T>
bool MedianFillFilter<T>::update(const T& mapIn, T& mapOut) {
  // Copy input map and add new layer to it.
  mapOut = mapIn;
  if (!mapOut.exists(outputLayer_)) {
    mapOut.add(outputLayer_);
  }
  mapOut.convertToDefaultStartIndex();
  const grid_map::Matrix inputMap = mapOut[inputLayer_];
  grid_map::Matrix& outputMap = mapOut[outputLayer_];
  grid_map::Matrix& varianceMap = mapOut["variance"];

  const auto radiusInPixels = static_cast<size_t>(fillHoleRadius_ / mapIn.getResolution());
  const auto existingValueRadiusInPixels = static_cast<size_t>(existingValueRadius_ / mapIn.getResolution());
  const grid_map::Index& bufferSize = mapOut.getSize();

  unsigned int numNans = 0;
  // Iterate through the entire GridMap and update NaN values with the median.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    const auto& inputValue = inputMap(index(0), index(1));
    auto& outputValue = outputMap(index(0), index(1));
    auto& variance = varianceMap(index(0), index(1));
    if (not std::isfinite(inputValue)) {  // Fill the NaN input value with the median.
      outputValue = getMedian(inputMap, index, radiusInPixels, bufferSize);
      numNans++;
    } else if (filterExistingValues_) {  // Value is already finite. Optionally add some filtering.
      outputValue = getMedian(inputMap, index, existingValueRadiusInPixels, bufferSize);
    } else {  // Dont do any filtering, just take the input value.
      outputValue = inputValue;
    }
  }
  ROS_DEBUG_STREAM("Median fill filter " << this->getName() << " observed " << numNans << " Nans in input layer!");
  mapOut.setBasicLayers({});
  return true;
}

}  // namespace grid_map

// Explicitly define the specialization for GridMap to have the filter implementation available for testing.
template class grid_map::MedianFillFilter<grid_map::GridMap>;
// Export the filter.
PLUGINLIB_EXPORT_CLASS(grid_map::MedianFillFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)