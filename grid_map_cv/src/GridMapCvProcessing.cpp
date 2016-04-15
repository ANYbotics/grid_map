/*
 * GridMapCvProcessing.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_cv/GridMapCvProcessing.hpp"
#include "grid_map_cv/GridMapCvConverter.hpp"

namespace grid_map {

GridMapCvProcessing::GridMapCvProcessing()
{
}

GridMapCvProcessing::~GridMapCvProcessing()
{
}

bool GridMapCvProcessing::changeResolution(const grid_map::GridMap& gridMapSource,
                             grid_map::GridMap& gridMapResult,
                             const double resolution,
                             const int interpolationAlgorithm)
{
  const double sizeFactor = gridMapSource.getResolution() / resolution;
  bool firstLayer = true;
  for (const auto& layer : gridMapSource.getLayers()) {
    cv::Mat imageSource, imageResult;
    const float minValue = gridMapSource.get(layer).minCoeffOfFinites();
    const float maxValue = gridMapSource.get(layer).maxCoeffOfFinites();
    const bool hasNaN = gridMapSource.get(layer).hasNaN();
    bool result;
    if (hasNaN) {
      result = GridMapCvConverter::toImage<unsigned short, 4>(gridMapSource, layer, CV_16UC4, minValue, maxValue, imageSource);
    } else {
      result = GridMapCvConverter::toImage<unsigned short, 1>(gridMapSource, layer, CV_16UC1, minValue, maxValue, imageSource);
    }
    if (!result) return false;
    cv::resize(imageSource, imageResult, cv::Size(0.0, 0.0), sizeFactor, sizeFactor, interpolationAlgorithm);
    if (firstLayer) {
      if (!GridMapCvConverter::initializeFromImage(imageResult, resolution, gridMapResult, gridMapSource.getPosition()))
        return false;
      firstLayer = false;
    }
    if (hasNaN) {
      result = GridMapCvConverter::addLayerFromImage<unsigned short, 4>(imageResult, layer, gridMapResult, minValue, maxValue);
    } else {
      result = GridMapCvConverter::addLayerFromImage<unsigned short, 1>(imageResult, layer, gridMapResult, minValue, maxValue);
    }
    if (!result) return false;
  }
  gridMapResult.setFrameId(gridMapSource.getFrameId());
  gridMapResult.setTimestamp(gridMapSource.getTimestamp());
  gridMapResult.setBasicLayers(gridMapSource.getBasicLayers());
  gridMapResult.setStartIndex(gridMapSource.getStartIndex());
  return true;
}

} /* namespace */
