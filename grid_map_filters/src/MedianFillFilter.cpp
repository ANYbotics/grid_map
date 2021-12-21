/*
 * MedianFillFilter.cpp
 *
 *  Created on: September 7, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MedianFillFilter.hpp"

#include <cmath>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace filters;

namespace grid_map {

MedianFillFilter::MedianFillFilter()
    : fillHoleRadius_(0.05), existingValueRadius_(0.0), filterExistingValues_(false), numErodeDilationIterations_(4), debug_(false) {}

MedianFillFilter::~MedianFillFilter() = default;

bool MedianFillFilter::configure() {
  if (!FilterBase::getParam(std::string("fill_hole_radius"), fillHoleRadius_)) {
    ROS_ERROR("Median filter did not find parameter fill_hole_radius.");
    return false;
  }

  if (fillHoleRadius_ < 0.0) {
    ROS_ERROR("fill_hole_radius must be greater than zero.");
    return false;
  }

  ROS_DEBUG("fill_hole_radius = %f.", fillHoleRadius_);

  if (!FilterBase::getParam(std::string("filter_existing_values"), filterExistingValues_)) {
    ROS_INFO("Median filter did not find parameter filter_existing_values. Not filtering existing values.");
    filterExistingValues_ = false;
  }

  ROS_DEBUG("Filter_existing_values = %s.", filterExistingValues_ ? "true" : "false");

  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Median filter did not find parameter `input_layer`.");
    return false;
  }

  if (!FilterBase::getParam(std::string("num_erode_dilation_iterations"), numErodeDilationIterations_)) {
    ROS_ERROR("Median filter did not find parameter `num_erode_dilation_iterations`.");
    return false;
  }

  if (filterExistingValues_) {
    if (!FilterBase::getParam(std::string("existing_value_radius"), existingValueRadius_)) {
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

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Median filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("Median output layer = %s.", outputLayer_.c_str());

  if (!FilterBase::getParam(std::string("fill_mask_layer"), fillMaskLayer_)) {
    ROS_ERROR("Median filter did not find parameter `fill_mask_layer`.");
    return false;
  }

  ROS_DEBUG("Median fill mask layer = %s.", fillMaskLayer_.c_str());

  if (!FilterBase::getParam(std::string("debug"), debug_)) {
    ROS_INFO("Median filter did not find parameter debug. Disabling debug output.");
    debug_ = false;
  }

  ROS_DEBUG("Debug mode= %s.", debug_ ? "true" : "false");

  if (debug_ && !FilterBase::getParam(std::string("debug_infill_mask_layer"), debugInfillMaskLayer_)) {
    ROS_ERROR("Median filter did not find parameter `debug_infill_mask_layer`.");
    return false;
  }

  ROS_DEBUG("Median debug infill mask layer = %s.", debugInfillMaskLayer_.c_str());

  return true;
}

bool MedianFillFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  // Copy input map and add new layer to it.
  mapOut = mapIn;
  if (!mapOut.exists(outputLayer_)) {
    mapOut.add(outputLayer_);
  }

  mapOut.convertToDefaultStartIndex();

  // Avoid hash map lookups afterwards. I.e, get data matrices as references.
  Matrix inputMap{mapOut[inputLayer_]};  // copy by value to do filtering first.
  Matrix& outputMap{mapOut[outputLayer_]};

  // Check if mask is already computed from a previous iteration.
  Eigen::MatrixXf shouldFill;
  if (std::find(mapOut.getLayers().begin(), mapOut.getLayers().end(), fillMaskLayer_) == mapOut.getLayers().end()) {
    shouldFill = computeAndAddFillMask(inputMap, mapOut);
  } else {  // The mask already exists, retrieve it from a previous iteration.
    shouldFill = mapOut[fillMaskLayer_];
  }

  const size_t radiusInPixels{static_cast<size_t>(fillHoleRadius_ / mapIn.getResolution())};
  const size_t existingValueRadiusInPixels{static_cast<size_t>(existingValueRadius_ / mapIn.getResolution())};
  const Index& bufferSize{mapOut.getSize()};
  unsigned int numNans{0u};
  // Iterate through the entire GridMap and update NaN values with the median.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    const auto& inputValue{inputMap(index(0), index(1))};
    const float& shouldFillThisCell{shouldFill(index(0), index(1))};
    auto& outputValue{outputMap(index(0), index(1))};
    if (!std::isfinite(inputValue) && (shouldFillThisCell != 0.0f)) {  // Fill the NaN input value with the median.
      outputValue = getMedian(inputMap, index, radiusInPixels, bufferSize);
      numNans++;
    } else if (filterExistingValues_ && (shouldFillThisCell != 0.0f)) {  // Value is already finite. Optionally add some filtering.
      outputValue = getMedian(inputMap, index, existingValueRadiusInPixels, bufferSize);
    } else {  // Dont do any filtering, just take the input value.
      outputValue = inputValue;
    }
  }
  ROS_DEBUG_STREAM("Median fill filter " << this->getName() << " observed " << numNans << " Nans in input layer!");
  // By removing all basic layers the selected layer will always be visualized, otherwise isValid will also check for the basic layers
  // and hide infilled values where the corresponding basic layers are still NAN.
  mapOut.setBasicLayers({});
  return true;
}

float MedianFillFilter::getMedian(Eigen::Ref<const Matrix> inputMap, const Index& centerIndex, const size_t radiusInPixels,
                                  const Size bufferSize) {
  // Bound the median window to the GridMap boundaries. Calculate the neighbour patch.
  Index topLeftIndex{centerIndex - Index(radiusInPixels, radiusInPixels)};
  Index bottomRightIndex{centerIndex + Index(radiusInPixels, radiusInPixels)};
  boundIndexToRange(topLeftIndex, bufferSize);
  boundIndexToRange(bottomRightIndex, bufferSize);
  const Index neighbourPatchSize{bottomRightIndex - topLeftIndex + Index{1, 1}};

  // Extract local neighbourhood.
  const auto& neighbourhood{inputMap.block(topLeftIndex(0), topLeftIndex(1), neighbourPatchSize(0), neighbourPatchSize(1))};

  const size_t cols{static_cast<size_t>(neighbourhood.cols())};

  std::vector<float> cellValues;
  cellValues.reserve(neighbourhood.rows() * neighbourhood.cols());

  for (Eigen::Index row = 0; row < neighbourhood.rows(); ++row) {
    const auto& currentRow{neighbourhood.row(row)};
    for (size_t col = 0; col < cols; ++col) {
      const float& cellValue{currentRow[col]};
      if (std::isfinite(cellValue)) {  // Calculate the median of the finite neighbours.
        cellValues.emplace_back(cellValue);
      }
    }
  }

  if (cellValues.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  } else {  // Compute the median of the finite values in the neighbourhood.
    std::nth_element(cellValues.begin(), cellValues.begin() + cellValues.size() / 2, cellValues.end());
    return cellValues[cellValues.size() / 2];
  }
}

Eigen::MatrixXf MedianFillFilter::computeAndAddFillMask(const Eigen::MatrixXf& inputMap, GridMap& mapOut) {
  Eigen::MatrixXf shouldFill;
  // Precompute mask of valid height values
  using MaskMatrix = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
  const MaskMatrix isValid{inputMap.array().unaryExpr([&](float v) { return std::isfinite(v); })};

  // Remove sparse valid values and fill holes.
  cv::Mat isValidCV;
  cv::eigen2cv(isValid, isValidCV);
  cv::Mat_<bool> isValidOutlierRemoved{cleanedMask(isValidCV)};
  cv::Mat shouldFillCV{fillHoles(isValidOutlierRemoved, numErodeDilationIterations_)};

  // Outlier removed mask to eigen.
  if (debug_) {
    addCvMatAsLayer(mapOut, isValidOutlierRemoved, debugInfillMaskLayer_);
  }

  // Convert to eigen and add to the map.
  cv::cv2eigen(shouldFillCV, shouldFill);
  mapOut.add(fillMaskLayer_, shouldFill);

  return shouldFill;
}

cv::Mat_<bool> MedianFillFilter::cleanedMask(const cv::Mat_<bool>& inputMask) {
  auto element{cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1))};

  cv::Mat_<bool> cleanedInputMask(inputMask.size(), false);

  // Erode then dilate to remove sparse points
  cv::dilate(inputMask, cleanedInputMask, element);
  cv::erode(cleanedInputMask, cleanedInputMask, element);
  cv::erode(inputMask, cleanedInputMask, element);
  cv::dilate(cleanedInputMask, cleanedInputMask, element);

  return cleanedInputMask;
}

cv::Mat_<bool> MedianFillFilter::fillHoles(const cv::Mat_<bool>& isValidMask, const size_t numDilationClosingIterations) {
  auto element{cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1))};
  cv::Mat_<bool> holesFilledMask(isValidMask.size(), false);
  // Remove holes in the mask by morphological closing.
  cv::dilate(isValidMask, holesFilledMask, element);
  for (size_t iteration = 1; iteration < numDilationClosingIterations; iteration++) {
    cv::dilate(holesFilledMask, holesFilledMask, element);
  }
  for (size_t iteration = 0; iteration < numDilationClosingIterations; iteration++) {
    cv::erode(holesFilledMask, holesFilledMask, element);
  }

  return holesFilledMask;
}

void MedianFillFilter::addCvMatAsLayer(GridMap& gridMap, const cv::Mat& cvLayer, const std::string& layerName) {
  Eigen::MatrixXf tmpEigenMatrix;
  cv::cv2eigen(cvLayer, tmpEigenMatrix);
  gridMap.add(layerName, tmpEigenMatrix);
}

}  // namespace grid_map
