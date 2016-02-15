/*
 * GridMapRosConverter.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMapRosConverter.hpp"
#include "grid_map/GridMapMsgHelpers.hpp"
#include <grid_map_core/grid_map_core.hpp>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// STL
#include <limits>
#include <algorithm>
#include <vector>

using namespace std;
using namespace Eigen;

namespace grid_map {

GridMapRosConverter::GridMapRosConverter()
{
}

GridMapRosConverter::~GridMapRosConverter()
{
}

bool GridMapRosConverter::fromMessage(const grid_map_msgs::GridMap& message, grid_map::GridMap& gridMap)
{
  gridMap.setTimestamp(message.info.header.stamp.toNSec());
  gridMap.setFrameId(message.info.header.frame_id);
  gridMap.setGeometry(Length(message.info.length_x, message.info.length_y), message.info.resolution,
                      Position(message.info.pose.position.x, message.info.pose.position.y));

  if (message.layers.size() != message.data.size()) {
    ROS_ERROR("Different number of layers and data in grid map message.");
    return false;
  }

  for (unsigned int i = 0; i < message.layers.size(); i++) {
    Matrix data;
    multiArrayMessageCopyToMatrixEigen(message.data[i], data); // TODO Could we use the data mapping (instead of copying) method here?
    // TODO Check if size is good.   size_ << getRows(message.data[0]), getCols(message.data[0]);
    gridMap.add(message.layers[i], data);
  }

  gridMap.setBasicLayers(message.basic_layers);
  gridMap.setStartIndex(Index(message.outer_start_index, message.inner_start_index));
  return true;
}

void GridMapRosConverter::toMessage(const grid_map::GridMap& gridMap, grid_map_msgs::GridMap& message)
{
  toMessage(gridMap, gridMap.getLayers(), message);
}

void GridMapRosConverter::toMessage(const grid_map::GridMap& gridMap, const std::vector<std::string>& layers,
                      grid_map_msgs::GridMap& message)
{
  message.info.header.stamp.fromNSec(gridMap.getTimestamp());
  message.info.header.frame_id = gridMap.getFrameId();
  message.info.resolution = gridMap.getResolution();
  message.info.length_x = gridMap.getLength().x();
  message.info.length_y = gridMap.getLength().y();
  message.info.pose.position.x = gridMap.getPosition().x();
  message.info.pose.position.y = gridMap.getPosition().y();
  message.info.pose.position.z = 0.0;
  message.info.pose.orientation.x = 0.0;
  message.info.pose.orientation.y = 0.0;
  message.info.pose.orientation.z = 0.0;
  message.info.pose.orientation.w = 1.0;

  message.layers = layers;
  message.basic_layers = gridMap.getBasicLayers();

  message.data.clear();
  for (const auto& layer : layers) {
    std_msgs::Float32MultiArray dataArray;
    matrixEigenCopyToMultiArrayMessage(gridMap.get(layer), dataArray);
    message.data.push_back(dataArray);
  }

  message.outer_start_index = gridMap.getStartIndex()(0);
  message.inner_start_index = gridMap.getStartIndex()(1);
}

void GridMapRosConverter::toPointCloud(const grid_map::GridMap& gridMap,
                                       const std::string& pointLayer,
                                       sensor_msgs::PointCloud2& pointCloud)
{
  toPointCloud(gridMap, gridMap.getLayers(), pointLayer, pointCloud);
}

void GridMapRosConverter::toPointCloud(const grid_map::GridMap& gridMap,
                                       const std::vector<std::string>& layers,
                                       const std::string& pointLayer,
                                       sensor_msgs::PointCloud2& pointCloud)
{
  // Header.
  pointCloud.header.frame_id = gridMap.getFrameId();
  pointCloud.header.stamp.fromNSec(gridMap.getTimestamp());
  pointCloud.is_dense = false;

  // Fields.
  std::vector <std::string> fieldNames;

  for (const auto& layer : layers) {
    if (layer == pointLayer) {
      fieldNames.push_back("x");
      fieldNames.push_back("y");
      fieldNames.push_back("z");
    } else if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  pointCloud.fields.clear();
  pointCloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (auto& name : fieldNames) {
    sensor_msgs::PointField point_field;
    point_field.name = name;
    point_field.count = 1;
    point_field.datatype = sensor_msgs::PointField::FLOAT32;
    point_field.offset = offset;
    pointCloud.fields.push_back(point_field);
    offset = offset + point_field.count * 4;  // 4 for sensor_msgs::PointField::FLOAT32
  }

  // Resize.
  size_t nPoints = gridMap.getSize().prod();
  pointCloud.height = 1;
  pointCloud.width = nPoints;
  pointCloud.point_step = offset;
  pointCloud.row_step = pointCloud.width * pointCloud.point_step;
  pointCloud.data.resize(pointCloud.height * pointCloud.row_step);

  // Points.
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> fieldIterators;
  for (auto& name : fieldNames) {
    fieldIterators.insert(
        std::pair<std::string, sensor_msgs::PointCloud2Iterator<float>>(
            name, sensor_msgs::PointCloud2Iterator<float>(pointCloud, name)));
  }

  GridMapIterator mapIterator(gridMap);

  for (size_t i = 0; i < nPoints; ++i) {
    Position3 position;
    position.setConstant(NAN);
    gridMap.getPosition3(pointLayer, *mapIterator, position);

    for (auto& iterator : fieldIterators) {
      if (iterator.first == "x") {
        *iterator.second = (float) position.x();
      } else if (iterator.first == "y") {
        *iterator.second = (float) position.y();
      } else if (iterator.first == "z") {
        *iterator.second = (float) position.z();
      } else if (iterator.first == "rgb") {
        *iterator.second = gridMap.at("color", *mapIterator);
      } else {
        *iterator.second = gridMap.at(iterator.first, *mapIterator);
      }
    }

    ++mapIterator;
    for (auto& iterator : fieldIterators) {
      ++iterator.second;
    }
  }
}

bool GridMapRosConverter::fromOccupancyGrid(const nav_msgs::OccupancyGrid& occupancyGrid,
                                            const std::string& layer, grid_map::GridMap& gridMap)
{
  const Size size(occupancyGrid.info.width, occupancyGrid.info.height);
  const double resolution = occupancyGrid.info.resolution;
  const Length length = resolution * size.cast<double>();
  const string& frameId = occupancyGrid.header.frame_id;
  Position position(occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y);
  // Different conventions of center of map.
  position += 0.5 * length.matrix();

  const auto& orientation = occupancyGrid.info.origin.orientation;
  if (orientation.w != 1.0 && !(orientation.x == 0 && orientation.y == 0
      && orientation.z == 0 && orientation.w == 0)) {
    ROS_WARN_STREAM("Conversion of occupancy grid: Grid maps do not support orientation.");
    ROS_INFO_STREAM("Orientation of occupancy grid: " << endl << occupancyGrid.info.origin.orientation);
    return false;
  }

  if (size.prod() != occupancyGrid.data.size()) {
    ROS_WARN_STREAM("Conversion of occupancy grid: Size of data does not correspond to width * height.");
    return false;
  }

  if ((gridMap.getSize() != size).any() || gridMap.getResolution() != resolution
      || (gridMap.getLength() != length).any() || gridMap.getPosition() != position
      || gridMap.getFrameId() != frameId || !gridMap.getStartIndex().isZero()) {
    gridMap.setTimestamp(occupancyGrid.header.stamp.toNSec());
    gridMap.setFrameId(frameId);
    gridMap.setGeometry(length, resolution, position);
  }

  // Reverse iteration is required because of different conventions
  // between occupancy grid and grid map.
  grid_map::Matrix data(size(0), size(1));
  for (std::vector<int8_t>::const_reverse_iterator iterator = occupancyGrid.data.rbegin();
      iterator != occupancyGrid.data.rend(); ++iterator) {
    size_t i = std::distance(occupancyGrid.data.rbegin(), iterator);
    data(i) = *iterator != -1 ? *iterator : NAN;
  }

  gridMap.add(layer, data);
  return true;
}

void GridMapRosConverter::toOccupancyGrid(const grid_map::GridMap& gridMap,
                                          const std::string& layer, float dataMin, float dataMax,
                                          nav_msgs::OccupancyGrid& occupancyGrid)
{
  occupancyGrid.header.frame_id = gridMap.getFrameId();
  occupancyGrid.header.stamp.fromNSec(gridMap.getTimestamp());
  occupancyGrid.info.map_load_time = occupancyGrid.header.stamp;  // Same as header stamp as we do not load the map.
  occupancyGrid.info.resolution = gridMap.getResolution();
  occupancyGrid.info.width = gridMap.getSize()(0);
  occupancyGrid.info.height = gridMap.getSize()(1);
  Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  occupancyGrid.info.origin.position.x = position.x();
  occupancyGrid.info.origin.position.y = position.y();
  occupancyGrid.info.origin.position.z = 0.0;
  occupancyGrid.info.origin.orientation.x = 0.0;
  occupancyGrid.info.origin.orientation.y = 0.0;
  occupancyGrid.info.origin.orientation.z = 0.0;
  occupancyGrid.info.origin.orientation.w = 1.0;
  size_t nCells = gridMap.getSize().prod();
  occupancyGrid.data.resize(nCells);

  // Occupancy probabilities are in the range [0,100]. Unknown is -1.
  const float cellMin = 0;
  const float cellMax = 100;
  const float cellRange = cellMax - cellMin;

  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    float value = (gridMap.at(layer, *iterator) - dataMin) / (dataMax - dataMin);
    if (isnan(value))
      value = -1;
    else
      value = cellMin + min(max(0.0f, value), 1.0f) * cellRange;
    size_t index = getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
    // Reverse cell order because of different conventions between occupancy grid and grid map.
    occupancyGrid.data[nCells - index - 1] = value;
  }
}

void GridMapRosConverter::toGridCells(const grid_map::GridMap& gridMap, const std::string& layer,
                                      float lowerThreshold, float upperThreshold,
                                      nav_msgs::GridCells& gridCells)
{
  gridCells.header.frame_id = gridMap.getFrameId();
  gridCells.header.stamp.fromNSec(gridMap.getTimestamp());
  gridCells.cell_width = gridMap.getResolution();
  gridCells.cell_height = gridMap.getResolution();

  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    if (!gridMap.isValid(*iterator, layer)) continue;
    if (gridMap.at(layer, *iterator) >= lowerThreshold
        && gridMap.at(layer, *iterator) <= upperThreshold) {
      Position position;
      gridMap.getPosition(*iterator, position);
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      gridCells.cells.push_back(point);
    }
  }
}

bool GridMapRosConverter::initializeFromImage(const sensor_msgs::Image& image,
                                              const double resolution, grid_map::GridMap& gridMap, const grid_map::Position& position)
{
  const double lengthX = resolution * image.height;
  const double lengthY = resolution * image.width;
  Length length(lengthX, lengthY);
  gridMap.setGeometry(length, resolution, position);
  gridMap.setFrameId(image.header.frame_id);
  gridMap.setTimestamp(image.header.stamp.toNSec());
  return true;
}

bool GridMapRosConverter::addLayerFromImage(const sensor_msgs::Image& image,
                                            const std::string& layer, grid_map::GridMap& gridMap,
                                            const double lowerValue, const double upperValue)
{
  cv_bridge::CvImagePtr cvPtrAlpha, cvPtrMono;

  // If alpha channel exist, read it.
  if (image.encoding == sensor_msgs::image_encodings::BGRA8
      || image.encoding == sensor_msgs::image_encodings::BGRA16) {
    try {
      cvPtrAlpha = cv_bridge::toCvCopy(image, image.encoding);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
  }

  unsigned int depth;
  // Convert color image to grayscale.
  try {
    if (image.encoding == sensor_msgs::image_encodings::BGRA8
        || image.encoding == sensor_msgs::image_encodings::BGR8
        || image.encoding == sensor_msgs::image_encodings::MONO8) {
      cvPtrMono = cv_bridge::toCvCopy(image,
                                      sensor_msgs::image_encodings::MONO8);
      depth = std::pow(2, 8);
      ROS_DEBUG("Color image converted to mono8");
    } else if (image.encoding == sensor_msgs::image_encodings::BGRA16
        || image.encoding == sensor_msgs::image_encodings::BGR16
        || image.encoding == sensor_msgs::image_encodings::MONO16) {
      cvPtrMono = cv_bridge::toCvCopy(image,
                                      sensor_msgs::image_encodings::MONO16);
      depth = std::pow(2, 16);
      ROS_DEBUG("Color image converted to mono16");
    } else {
      ROS_ERROR("Expected BGR, BGRA, or MONO image encoding.");
      return false;
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  gridMap.add(layer);

  if (gridMap.getSize()(0) != image.height
      || gridMap.getSize()(1) != image.width) {
    ROS_ERROR("Image size does not correspond to grid map size!");
    return false;
  }

  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    // Set transparent values.
    if (image.encoding == sensor_msgs::image_encodings::BGRA8) {
      const auto& cvAlpha = cvPtrAlpha->image.at<cv::Vec4b>((*iterator)(0),
                                                            (*iterator)(1));
      unsigned int alpha = cvAlpha[3];
      if (cvAlpha[3] < depth / 2)
        continue;
    }
    if (image.encoding == sensor_msgs::image_encodings::BGRA16) {
      const auto& cvAlpha = cvPtrAlpha->image.at<cv::Vec<uchar, 8>>(
          (*iterator)(0), (*iterator)(1));
      int alpha = (cvAlpha[6] << 8) + cvAlpha[7];
      if (alpha < depth / 2)
        continue;
    }

    // Compute height.
    unsigned int grayValue;
    if (depth == std::pow(2, 8)) {
      uchar cvGrayscale = cvPtrMono->image.at<uchar>((*iterator)(0),
                                                     (*iterator)(1));
      grayValue = cvGrayscale;
    }
    if (depth == std::pow(2, 16)) {
      const auto& cvGrayscale = cvPtrMono->image.at<cv::Vec2b>((*iterator)(0),
                                                               (*iterator)(1));
      grayValue = (cvGrayscale[0] << 8) + cvGrayscale[1];
    }

    double height = lowerValue
        + (upperValue - lowerValue) * ((double) grayValue / (double) depth);
    gridMap.at(layer, *iterator) = height;
  }

  return true;
}

bool GridMapRosConverter::addColorLayerFromImage(const sensor_msgs::Image& image,
                                                 const std::string& layer,
                                                 grid_map::GridMap& gridMap)
{
  cv_bridge::CvImagePtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvCopy(image, image.encoding);
//    cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); // FixMe
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  gridMap.add(layer);

  if (gridMap.getSize()(0) != image.height || gridMap.getSize()(1) != image.width) {
    ROS_ERROR("Image size does not correspond to grid map size!");
    return false;
  }

  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    const auto& cvColor = cvPtr->image.at<cv::Vec3b>((*iterator)(0), (*iterator)(1));
    Eigen::Vector3i colorVector;
    // TODO Add cases for different image encodings.
    colorVector(2) = cvColor[0];  // From BGR to RGB.
    colorVector(1) = cvColor[1];
    colorVector(0) = cvColor[2];
    colorVectorToValue(colorVector, gridMap.at(layer, *iterator));
  }

  return true;
}

bool GridMapRosConverter::toCvImage(const grid_map::GridMap& gridMap, const std::string& layer,
                                    cv::Mat& cvImage, const float dataMin, const float dataMax)
{
  if (gridMap.getSize()(0) > 0 && gridMap.getSize()(1) > 0) {
    // Initialize blank image.
    cvImage = cv::Mat::zeros(gridMap.getSize()(0), gridMap.getSize()(1), CV_8UC4);
  } else {
    ROS_ERROR("Invalid grid map?");
    return false;
  }

  // Clamp outliers.
  grid_map::GridMap map = gridMap;
  map.get(layer) = map.get(layer).unaryExpr(grid_map::Clamp<float>(dataMin, dataMax));

  // Find upper and lower values.
  float lowerValue = map.get(layer).minCoeffOfFinites();
  float upperValue = map.get(layer).maxCoeffOfFinites();

  uchar imageMax = std::numeric_limits<unsigned char>::max();
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (map.isValid(*iterator, layer)) {
      float value = map.at(layer, *iterator);
      uchar imageValue = (uchar)(((value - lowerValue) / (upperValue - lowerValue)) * (float)imageMax);
      grid_map::Index imageIndex(iterator.getUnwrappedIndex());
      cvImage.at<cv::Vec<uchar, 4>>(imageIndex(1), imageIndex(0))[0] = imageValue;
      cvImage.at<cv::Vec<uchar, 4>>(imageIndex(1), imageIndex(0))[1] = imageValue;
      cvImage.at<cv::Vec<uchar, 4>>(imageIndex(1), imageIndex(0))[2] = imageValue;
      cvImage.at<cv::Vec<uchar, 4>>(imageIndex(1), imageIndex(0))[3] = imageMax;
    }
  }

  return true;
}

bool GridMapRosConverter::saveToBag(const grid_map::GridMap& gridMap, const std::string& pathToBag,
                                    const std::string& topic)
{
  grid_map_msgs::GridMap message;
  toMessage(gridMap, message);
  ros::Time time(gridMap.getTimestamp());

  if (!time.isValid() || time.isZero()) {
    if (!ros::Time::isValid()) ros::Time::init();
    time = ros::Time::now();
  }

  rosbag::Bag bag;
  bag.open(pathToBag, rosbag::bagmode::Write);
  bag.write(topic, time, message);
  bag.close();
  return true;
}

bool GridMapRosConverter::loadFromBag(const std::string& pathToBag, const std::string& topic,
                                      grid_map::GridMap& gridMap)
{
  rosbag::Bag bag;
  bag.open(pathToBag, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topic));

  bool isDataFound = false;
  for (const auto& messageInstance : view) {
    grid_map_msgs::GridMap::ConstPtr message = messageInstance.instantiate<grid_map_msgs::GridMap>();
    if (message != NULL) {
      fromMessage(*message, gridMap);
      isDataFound = true;
    } else {
      bag.close();
      ROS_WARN("Unable to load data from ROS bag.");
      return false;
    }
  }
  if (!isDataFound) ROS_WARN_STREAM("No data under the topic '" << topic << "' was found.");
  bag.close();
  return true;
}

} /* namespace */
