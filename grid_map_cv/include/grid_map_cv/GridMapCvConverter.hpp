/*
 * GridMapCvConverter.hpp
 *
 *  Created on: Apr 14, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// STD
#include <iostream>

namespace grid_map {

/*!
 * Conversions between grid maps and OpenCV images.
 */
class GridMapCvConverter
{
 public:
  /*!
   * Default constructor.
   */
  GridMapCvConverter();

  /*!
   * Destructor.
   */
  virtual ~GridMapCvConverter();

  /*!
   * Initializes the geometry of a grid map from an image. This changes
   * the geometry of the map and deletes all contents of the layers!
   * @param[in] image the image.
   * @param[in] resolution the desired resolution of the grid map [m/cell].
   * @param[out] gridMap the grid map to be initialized.
   * @param[in](optional) position the position of the grid map.
   * @return true if successful, false otherwise.
   */
  static bool initializeFromImage(const cv::Mat& image, const double resolution,
                                  grid_map::GridMap& gridMap, const grid_map::Position& position)
  {
    const double lengthX = resolution * image.rows;
    const double lengthY = resolution * image.cols;
    Length length(lengthX, lengthY);
    gridMap.setGeometry(length, resolution, position);
    return true;
  }

  /*!
   * Adds a layer with data from image.
   * @param[in] image the image to be added. If it is a color image
   * (bgr or bgra encoding), it will be transformed in a grayscale image.
   * @param[in] layer the layer that is filled with the image data.
   * @param[out] gridMap the grid map to be populated.
   * @param[in](optional) lowerValue value of the layer corresponding to black image pixels.
   * @param[in](optional) upperValue value of the layer corresponding to white image pixels.
   * @param[in](optional) alphaThreshold the threshold ([0.0, 1.0]) for the alpha value at which
   * cells in the grid map are marked as empty.
   * @return true if successful, false otherwise.
   */
  template<typename Type_, int NChannels_>
  static bool addLayerFromImage(const cv::Mat& image, const std::string& layer,
                                grid_map::GridMap& gridMap, const float lowerValue = 0.0,
                                const float upperValue = 1.0, const double alphaThreshold = 0.5)
  {
    if (gridMap.getSize()(0) != image.rows || gridMap.getSize()(1) != image.cols) {
      std::cerr << "Image size does not correspond to grid map size!" << std::endl;
      return false;
    }

    bool isColor = false;
    if (image.channels() >= 3) isColor = true;
    bool hasAlpha = false;
    if (image.channels() >= 4) hasAlpha = true;

    cv::Mat imageMono;
    if (isColor && !hasAlpha) {
      cv::cvtColor(image, imageMono, CV_BGR2GRAY);
    } else if (isColor && hasAlpha) {
      cv::cvtColor(image, imageMono, CV_BGRA2GRAY);
    } else if (!isColor && !hasAlpha){
      imageMono = image;
    } else {
      std::cerr << "Something went wrong when adding grid map layer form image!" << std::endl;
      return false;
    }

    const float mapValueDifference = upperValue - lowerValue;
    const float maxImageValue = (float)std::numeric_limits<Type_>::max();
    const Type_ alphaTreshold = (Type_)(alphaThreshold * std::numeric_limits<Type_>::max());

    gridMap.add(layer);
    grid_map::Matrix& data = gridMap[layer];

    for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);

      // Check for alpha layer.
      if (hasAlpha) {
        const Type_ alpha = image.at<cv::Vec<Type_, NChannels_>>(index(0), index(1))[NChannels_ - 1];
        if (alpha < alphaTreshold) continue;
      }

      // Compute value.
      const Type_ imageValue = imageMono.at<Type_>(index(0), index(1));
      const float mapValue = lowerValue + mapValueDifference * ((float) imageValue / maxImageValue);
      data(index(0), index(1)) = mapValue;
    }

    return true;
  };

  /*!
   * Creates a cv mat from a grid map layer.
   * This conversion sets the corresponding black and white pixel value to the
   * min. and max. data of the layer data.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[in] lowerValue the value of the layer corresponding to black image pixels.
   * @param[in] upperValue the value of the layer corresponding to white image pixels.
   * @param[out] image the image to be populated.
   * @return true if successful, false otherwise.
   */
  template<typename Type_, int NChannels_>
  static bool toImage(const grid_map::GridMap& gridMap, const std::string& layer,
                      const int encoding, cv::Mat& image)
  {
    const float minValue = gridMap.get(layer).minCoeffOfFinites();
    const float maxValue = gridMap.get(layer).maxCoeffOfFinites();
    return toImage<Type_, NChannels_>(gridMap, layer, encoding, minValue, maxValue, image);
  };

  /*!
   * Creates a cv mat from a grid map layer.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[in] lowerValue the value of the layer corresponding to black image pixels.
   * @param[in] upperValue the value of the layer corresponding to white image pixels.
   * @param[out] image the image to be populated.
   * @return true if successful, false otherwise.
   */
  template<typename Type_, int NChannels_>
  static bool toImage(const grid_map::GridMap& gridMap, const std::string& layer,
                      const int encoding, const float lowerValue, const float upperValue,
                      cv::Mat& image)
  {
    // Initialize image.
    if (gridMap.getSize()(0) > 0 && gridMap.getSize()(1) > 0) {
      image = cv::Mat::zeros(gridMap.getSize()(0), gridMap.getSize()(1), encoding);
    } else {
      std::cerr << "Invalid grid map?" << std::endl;
      return false;
    }

    // Get max image value.
    unsigned int imageMax = (unsigned int)std::numeric_limits<Type_>::max();

    // Clamp outliers.
    grid_map::GridMap map = gridMap;
    map.get(layer) = map.get(layer).unaryExpr(grid_map::Clamp<float>(lowerValue, upperValue));
    const grid_map::Matrix& data = gridMap[layer];

    // Convert to image.
    bool isColor = false;
    if (image.channels() >= 3) isColor = true;
    bool hasAlpha = false;
    if (image.channels() >= 4) hasAlpha = true;

    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      if (std::isfinite(data(index(0), index(1)))) {
        const float& value = data(index(0), index(1));
        const Type_ imageValue = (Type_) (((value - lowerValue) / (upperValue - lowerValue)) * (float) imageMax);
        const Index imageIndex(iterator.getUnwrappedIndex());
        unsigned int channel = 0;
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[channel] = imageValue;

        if (isColor) {
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
        }
        if (hasAlpha) {
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageMax;
        }
      }
    }

    return true;
  };

  template<typename Type_, int NChannels_>
  static bool toWeightedImage(const grid_map::GridMap& gridMap, const std::string& layer, const std::string& weight,
                      const int encoding, cv::Mat& image)
  {
    const float minValue = gridMap.get(layer).minCoeffOfFinites();
    const float maxValue = gridMap.get(layer).maxCoeffOfFinites();
    return toWeightedImage<Type_, NChannels_>(gridMap, layer, weight, encoding, minValue, maxValue, image);
  };

  template<typename Type_, int NChannels_>
  static bool toWeightedImage(const grid_map::GridMap& gridMap, const std::string& layer, const std::string& weight,
                      const int encoding, const float lowerValue, const float upperValue,
                      cv::Mat& image)
  {
    // Initialize image.
    if (gridMap.getSize()(0) > 0 && gridMap.getSize()(1) > 0) {
      image = cv::Mat::zeros(gridMap.getSize()(0), gridMap.getSize()(1), encoding);
    } else {
      std::cerr << "Invalid grid map?" << std::endl;
      return false;
    }

    // Get max image value.
    unsigned int imageMax = (unsigned int)std::numeric_limits<Type_>::max();

    const float minWeight = gridMap.get(weight).minCoeffOfFinites();
    const float maxWeight = gridMap.get(weight).maxCoeffOfFinites();

    // Clamp outliers.
    grid_map::GridMap map = gridMap;
    map.get(layer) = map.get(layer).unaryExpr(grid_map::Clamp<float>(lowerValue, upperValue));
    const grid_map::Matrix& data = gridMap[layer];
    const grid_map::Matrix& weights = gridMap[weight];

    // Convert to image.
    bool isColor = false;
    if (image.channels() >= 3) isColor = true;
    bool hasAlpha = false;
    if (image.channels() >= 4) hasAlpha = true;

    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      if (std::isfinite(data(index(0), index(1)))) {
        const float& value = data(index(0), index(1));
        const Type_ imageValue = (Type_) (((value - lowerValue) / (upperValue - lowerValue)) * (float) imageMax);
        const float& alpha = weights(index(0), index(1));
        const Type_ imageWeight = (Type_) (((alpha - minWeight) / (maxWeight - minWeight)) * (float) imageMax);
        const Index imageIndex(iterator.getUnwrappedIndex());
        unsigned int channel = 0;
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[channel] = imageValue;

        if (isColor) {
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
        }
        if (hasAlpha) {
          image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageWeight;
        }
      }
    }

    return true;
  };

};

} /* namespace */
