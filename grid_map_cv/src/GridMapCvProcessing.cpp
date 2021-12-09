/*
 * GridMapCvProcessing.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: Péter Fankhauser, Magnus Gärtner
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_cv/GridMapCvProcessing.hpp"
#include "grid_map_cv/GridMapCvConverter.hpp"

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

namespace grid_map {

GridMapCvProcessing::GridMapCvProcessing() {}

GridMapCvProcessing::~GridMapCvProcessing() {}

bool GridMapCvProcessing::changeResolution(const GridMap& gridMapSource, GridMap& gridMapResult, const double resolution,
                                           const int interpolationAlgorithm) {
  GridMap gridMapSourceCopy(gridMapSource);
  gridMapSourceCopy.convertToDefaultStartIndex();
  const double sizeFactor = gridMapSourceCopy.getResolution() / resolution;
  bool firstLayer = true;
  for (const auto& layer : gridMapSourceCopy.getLayers()) {
    cv::Mat imageSource, imageResult;
    const float minValue = gridMapSourceCopy.get(layer).minCoeffOfFinites();
    const float maxValue = gridMapSourceCopy.get(layer).maxCoeffOfFinites();
    const bool hasNaN = gridMapSourceCopy.get(layer).hasNaN();
    bool result;
    if (hasNaN) {
      result = GridMapCvConverter::toImage<unsigned short, 4>(gridMapSourceCopy, layer, CV_16UC4, minValue, maxValue, imageSource);
    } else {
      result = GridMapCvConverter::toImage<unsigned short, 1>(gridMapSourceCopy, layer, CV_16UC1, minValue, maxValue, imageSource);
    }
    if (!result) return false;
    cv::resize(imageSource, imageResult, cv::Size(0.0, 0.0), sizeFactor, sizeFactor, interpolationAlgorithm);
    if (firstLayer) {
      if (!GridMapCvConverter::initializeFromImage(imageResult, resolution, gridMapResult, gridMapSourceCopy.getPosition())) return false;
      firstLayer = false;
    }
    if (hasNaN) {
      result = GridMapCvConverter::addLayerFromImage<unsigned short, 4>(imageResult, layer, gridMapResult, minValue, maxValue);
    } else {
      result = GridMapCvConverter::addLayerFromImage<unsigned short, 1>(imageResult, layer, gridMapResult, minValue, maxValue);
    }
    if (!result) return false;
  }
  gridMapResult.setFrameId(gridMapSourceCopy.getFrameId());
  gridMapResult.setTimestamp(gridMapSourceCopy.getTimestamp());
  gridMapResult.setBasicLayers(gridMapSourceCopy.getBasicLayers());
  return true;
}

GridMap GridMapCvProcessing::getTransformedMap(GridMap&& gridMapSource, const Eigen::Isometry3d& transform,
                                               const std::string& heightLayerName, const std::string& newFrameId) {
  // Check if height layer is valid.
  if (!gridMapSource.exists(heightLayerName)) {
    throw std::out_of_range("GridMap::getTransformedMap(...) : No map layer '" + heightLayerName + "' available.");
  }

  // Check if transformation is z aligned.
  if (std::abs(transform(2, 2) - 1) >= 0.05) {
    throw std::invalid_argument("The given transform is not Z aligned!");
  }

  auto yawPitchRoll = transform.rotation().eulerAngles(2, 1, 0);  // Double check convention!
  double rotationAngle = yawPitchRoll.x() * 180 / CV_PI;
  if (std::abs(yawPitchRoll.y()) >= 3 && std::abs(yawPitchRoll.z()) >= 3) {  // Resolve yaw ambiguity in euler angles.
    rotationAngle += 180;
  }

  gridMapSource.convertToDefaultStartIndex();

  // Create the rotated gridMap.
  GridMap transformedMap(gridMapSource.getLayers());
  transformedMap.setBasicLayers(gridMapSource.getBasicLayers());
  transformedMap.setTimestamp(gridMapSource.getTimestamp());
  transformedMap.setFrameId(newFrameId);

  // openCV rotation parameters, initalized on first layer.
  cv::Mat imageRotationMatrix;
  cv::Rect2f boundingBox;

  bool firstLayer = true;
  for (const auto& layer : gridMapSource.getLayers()) {
    cv::Mat imageSource, imageResult;

    // From gridMap to openCV image. Assumes defaultStartIndex.
    cv::eigen2cv(gridMapSource[layer], imageSource);

    // Calculate transformation matrix and update geometry of the resulting grid map.
    if (firstLayer) {
      // Get rotation matrix for rotating the image around its center in pixel coordinates. See
      // https://answers.opencv.org/question/82708/rotation-center-confusion/
      cv::Point2f imageCenter =
          cv::Point2f((imageSource.cols - 1) / 2.0, (imageSource.rows - 1) / 2.0);

      imageRotationMatrix = cv::getRotationMatrix2D(imageCenter, rotationAngle, 1.0);
      boundingBox = cv::RotatedRect(cv::Point2f(0, 0), imageSource.size(), rotationAngle).boundingRect2f();

      // Adjust transformation matrix. See
      // https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326 and https://stackoverflow.com/questions/22041699/rotate-an-image-without-cropping-in-opencv-in-c
      imageRotationMatrix.at<double>(0, 2) += boundingBox.width / 2.0 - imageSource.cols / 2.0;
      imageRotationMatrix.at<double>(1, 2) += boundingBox.height / 2.0 - imageSource.rows / 2.0;

      // Calculate the new center of the gridMap.
      Position3 newCenter = transform * Position3{(gridMapSource.getPosition().x()), (gridMapSource.getPosition().y()), 0.0};

      // Set the size of the rotated gridMap.
      transformedMap.setGeometry({boundingBox.height * gridMapSource.getResolution(), boundingBox.width * gridMapSource.getResolution()},
                         gridMapSource.getResolution(), Position(newCenter.x(), newCenter.y()));
      firstLayer = false;
    }

    // Rotate the layer.
    imageResult = cv::Mat(boundingBox.size(), CV_32F, std::numeric_limits<double>::quiet_NaN());
    cv::warpAffine(imageSource, imageResult, imageRotationMatrix, boundingBox.size(), cv::INTER_NEAREST, cv::BORDER_TRANSPARENT);

    // Copy result into gridMapLayer. Assumes default start index.
    Matrix resultLayer;
    cv::cv2eigen(imageResult, resultLayer);
    transformedMap.add(layer, resultLayer);
  }

  // Add height translation.
  grid_map::Matrix heightLayer = transformedMap[heightLayerName];
  transformedMap[heightLayerName] = heightLayer.array() + transform.translation().z();

  return transformedMap;
}
}  // namespace grid_map
