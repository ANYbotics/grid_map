/*
 * GridMapRosConverter.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// ROS
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/time.h>

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

// STL
#include <limits>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include <memory>

#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

using namespace std;  // NOLINT
using namespace Eigen;  // NOLINT

namespace grid_map
{

GridMapRosConverter::GridMapRosConverter()
{
}

GridMapRosConverter::~GridMapRosConverter()
{
}

bool GridMapRosConverter::fromMessage(
  const grid_map_msgs::msg::GridMap & message,
  grid_map::GridMap & gridMap,
  const std::vector<std::string> & layers, bool copyBasicLayers,
  bool copyAllNonBasicLayers)
{
  gridMap.setTimestamp(rclcpp::Time(message.header.stamp).nanoseconds());
  gridMap.setFrameId(message.header.frame_id);
  gridMap.setGeometry(
    Length(message.info.length_x, message.info.length_y), message.info.resolution,
    Position(message.info.pose.position.x, message.info.pose.position.y));

  if (message.layers.size() != message.data.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "fromMessage"), "Different number of layers and data in grid map message.");
    return false;
  }

  // Copy non-basic layers.
  for (unsigned int i = 0u; i < message.layers.size(); ++i) {
    // check if layer should be copied.
    if (!copyAllNonBasicLayers &&
      std::find(layers.begin(), layers.end(), message.layers[i]) == layers.end())
    {
      continue;
    }

    // TODO(needs_assignment) Could we use the data mapping (instead of copying) method here?
    Matrix data;
    if (!multiArrayMessageCopyToMatrixEigen(message.data[i], data)) {
      return false;
    }

    // TODO(needs_assignment) Check if size is good.
    // size_ << getRows(message.data[0]), getCols(message.data[0]);
    gridMap.add(message.layers[i], data);
  }

  // Copy basic layers.
  if (copyBasicLayers) {
    gridMap.setBasicLayers(message.basic_layers);
  }

  gridMap.setStartIndex(Index(message.outer_start_index, message.inner_start_index));
  return true;
}

bool GridMapRosConverter::fromMessage(
  const grid_map_msgs::msg::GridMap & message,
  grid_map::GridMap & gridMap)
{
  return fromMessage(message, gridMap, std::vector<std::string>(), true, true);
}

std::unique_ptr<grid_map_msgs::msg::GridMap> GridMapRosConverter::toMessage(
  const grid_map::GridMap & gridMap)
{
  return toMessage(gridMap, gridMap.getLayers());
}

std::unique_ptr<grid_map_msgs::msg::GridMap> GridMapRosConverter::toMessage(
  const grid_map::GridMap & gridMap, const std::vector<std::string> & layers)
{
  auto message = std::make_unique<grid_map_msgs::msg::GridMap>();

  message->header.stamp = rclcpp::Time(gridMap.getTimestamp());
  message->header.frame_id = gridMap.getFrameId();
  message->info.resolution = gridMap.getResolution();
  message->info.length_x = gridMap.getLength().x();
  message->info.length_y = gridMap.getLength().y();
  message->info.pose.position.x = gridMap.getPosition().x();
  message->info.pose.position.y = gridMap.getPosition().y();
  message->info.pose.position.z = 0.0;
  message->info.pose.orientation.x = 0.0;
  message->info.pose.orientation.y = 0.0;
  message->info.pose.orientation.z = 0.0;
  message->info.pose.orientation.w = 1.0;

  message->layers = layers;
  message->basic_layers = gridMap.getBasicLayers();

  message->data.clear();
  for (const auto & layer : layers) {
    std_msgs::msg::Float32MultiArray dataArray;
    matrixEigenCopyToMultiArrayMessage(gridMap.get(layer), dataArray);
    message->data.push_back(dataArray);
  }

  message->outer_start_index = gridMap.getStartIndex()(0);
  message->inner_start_index = gridMap.getStartIndex()(1);

  return message;
}

void GridMapRosConverter::toPointCloud(
  const grid_map::GridMap & gridMap,
  const std::string & pointLayer,
  sensor_msgs::msg::PointCloud2 & pointCloud)
{
  toPointCloud(gridMap, gridMap.getLayers(), pointLayer, pointCloud);
}

void GridMapRosConverter::toPointCloud(
  const grid_map::GridMap & gridMap,
  const std::vector<std::string> & layers,
  const std::string & pointLayer,
  sensor_msgs::msg::PointCloud2 & pointCloud)
{
  // Header.
  pointCloud.header.frame_id = gridMap.getFrameId();
  pointCloud.header.stamp = rclcpp::Time(gridMap.getTimestamp());
  pointCloud.is_dense = false;

  // Fields.
  std::vector<std::string> fieldNames;

  for (const auto & layer : layers) {
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

  for (auto & name : fieldNames) {
    sensor_msgs::msg::PointField pointField;
    pointField.name = name;
    pointField.count = 1;
    pointField.datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointField.offset = offset;
    pointCloud.fields.push_back(pointField);
    offset = offset + pointField.count * 4;  // 4 for sensor_msgs::msg::PointField::FLOAT32
  }

  // Resize.
  size_t maxNumberOfPoints = gridMap.getSize().prod();
  pointCloud.height = 1;
  pointCloud.width = maxNumberOfPoints;
  pointCloud.point_step = offset;
  pointCloud.row_step = pointCloud.width * pointCloud.point_step;
  pointCloud.data.resize(pointCloud.height * pointCloud.row_step);

  // Points.
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> fieldIterators;
  for (auto & name : fieldNames) {
    fieldIterators.insert(
      std::pair<std::string, sensor_msgs::PointCloud2Iterator<float>>(
        name, sensor_msgs::PointCloud2Iterator<float>(pointCloud, name)));
  }

  GridMapIterator mapIterator(gridMap);
  const bool hasBasicLayers = gridMap.getBasicLayers().size() > 0;

  size_t realNumberOfPoints = 0;
  for (size_t i = 0; i < maxNumberOfPoints; ++i) {
    if (hasBasicLayers) {
      if (!gridMap.isValid(*mapIterator)) {
        ++mapIterator;
        continue;
      }
    }

    Position3 position;
    if (!gridMap.getPosition3(pointLayer, *mapIterator, position)) {
      ++mapIterator;
      continue;
    }

    for (auto & iterator : fieldIterators) {
      if (iterator.first == "x") {
        *iterator.second = static_cast<float>(position.x());
      } else if (iterator.first == "y") {
        *iterator.second = static_cast<float>(position.y());
      } else if (iterator.first == "z") {
        *iterator.second = static_cast<float>(position.z());
      } else if (iterator.first == "rgb") {
        *iterator.second = gridMap.at("color", *mapIterator);
      } else {
        *iterator.second = gridMap.at(iterator.first, *mapIterator);
      }
    }

    ++mapIterator;
    for (auto & iterator : fieldIterators) {
      ++iterator.second;
    }
    ++realNumberOfPoints;
  }

  if (realNumberOfPoints != maxNumberOfPoints) {
    pointCloud.width = realNumberOfPoints;
    pointCloud.row_step = pointCloud.width * pointCloud.point_step;
    pointCloud.data.resize(pointCloud.height * pointCloud.row_step);
  }
}

bool GridMapRosConverter::fromOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & occupancyGrid,
  const std::string & layer, grid_map::GridMap & gridMap)
{
  const Size size(occupancyGrid.info.width, occupancyGrid.info.height);
  const double resolution = occupancyGrid.info.resolution;
  const Length length = resolution * size.cast<double>();
  const string & frameId = occupancyGrid.header.frame_id;
  Position position(occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y);
  // Different conventions of center of map.
  position += 0.5 * length.matrix();

  const auto & orientation = occupancyGrid.info.origin.orientation;
  if (orientation.w != 1.0 && !(orientation.x == 0 && orientation.y == 0 &&
    orientation.z == 0 && orientation.w == 0))
  {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "fromOccupancyGrid"),
      "Conversion of occupancy grid: Grid maps do not support orientation.");
    RCLCPP_INFO(
      rclcpp::get_logger("fromOccupancyGrid"),
      "Orientation of occupancy grid: \n%s",
      geometry_msgs::msg::to_yaml(occupancyGrid.info.origin.orientation).c_str());
    return false;
  }

  if (static_cast<uint64_t>(size.prod()) != occupancyGrid.data.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("fromOccupancyGrid"),
      "Conversion of occupancy grid: Size of data does not correspond to width * height.");
    return false;
  }

  // TODO(needs_assignment): Split to `initializeFrom` and `from` as for Costmap2d.
  if ((gridMap.getSize() != size).any() || gridMap.getResolution() != resolution ||
    (gridMap.getLength() != length).any() || gridMap.getPosition() != position ||
    gridMap.getFrameId() != frameId || !gridMap.getStartIndex().isZero())
  {
    gridMap.setTimestamp(rclcpp::Time(occupancyGrid.header.stamp).nanoseconds());
    gridMap.setFrameId(frameId);
    gridMap.setGeometry(length, resolution, position);
  }

  // Reverse iteration is required because of different conventions
  // between occupancy grid and grid map.
  grid_map::Matrix data(size(0), size(1));
  for (std::vector<int8_t>::const_reverse_iterator iterator = occupancyGrid.data.rbegin();
    iterator != occupancyGrid.data.rend(); ++iterator)
  {
    size_t i = std::distance(occupancyGrid.data.rbegin(), iterator);
    data(i) = *iterator != -1 ? *iterator : NAN;
  }

  gridMap.add(layer, data);
  return true;
}

void GridMapRosConverter::toOccupancyGrid(
  const grid_map::GridMap & gridMap,
  const std::string & layer, float dataMin, float dataMax,
  nav_msgs::msg::OccupancyGrid & occupancyGrid)
{
  occupancyGrid.header.frame_id = gridMap.getFrameId();
  occupancyGrid.header.stamp = rclcpp::Time(gridMap.getTimestamp());
  // Same as header stamp as we do not load the map.
  occupancyGrid.info.map_load_time = occupancyGrid.header.stamp;
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
    if (isnan(value)) {
      value = -1;
    } else {
      value = cellMin + min(max(0.0f, value), 1.0f) * cellRange;
    }
    size_t index = getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
    // Reverse cell order because of different conventions between occupancy grid and grid map.
    occupancyGrid.data[nCells - index - 1] = value;
  }
}

bool GridMapRosConverter::fromCostmap(
  const nav2_msgs::msg::Costmap & costmap,
  const std::string & layer, grid_map::GridMap & gridMap)
{
  const Size size(costmap.metadata.size_x, costmap.metadata.size_y);
  const double resolution = costmap.metadata.resolution;
  const Length length = resolution * size.cast<double>();
  const string & frameId = costmap.header.frame_id;
  Position position(costmap.metadata.origin.position.x, costmap.metadata.origin.position.y);
  // Different conventions of center of map.
  position += 0.5 * length.matrix();

  const auto & orientation = costmap.metadata.origin.orientation;
  if (orientation.w != 1.0 && !(orientation.x == 0 && orientation.y == 0 &&
    orientation.z == 0 && orientation.w == 0))
  {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "fromcostmap"),
      "Conversion of costmap: Grid maps do not support orientation.");
    RCLCPP_INFO(
      rclcpp::get_logger("fromcostmap"),
      "Orientation of costmap: \n%s",
      geometry_msgs::msg::to_yaml(costmap.metadata.origin.orientation).c_str());
    return false;
  }

  if (static_cast<uint64_t>(size.prod()) != costmap.data.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("fromcostmap"),
      "Conversion of costmap: Size of data does not correspond to width * height.");
    return false;
  }

  // TODO(needs_assignment): Split to `initializeFrom` and `from` as for Costmap2d.
  if ((gridMap.getSize() != size).any() || gridMap.getResolution() != resolution ||
    (gridMap.getLength() != length).any() || gridMap.getPosition() != position ||
    gridMap.getFrameId() != frameId || !gridMap.getStartIndex().isZero())
  {
    gridMap.setTimestamp(rclcpp::Time(costmap.header.stamp).nanoseconds());
    gridMap.setFrameId(frameId);
    gridMap.setGeometry(length, resolution, position);
  }

  // Reverse iteration is required because of different conventions
  // between costmap and grid map.
  grid_map::Matrix data(size(0), size(1));
  for (std::vector<uint8_t>::const_reverse_iterator iterator = costmap.data.rbegin();
    iterator != costmap.data.rend(); ++iterator)
  {
    size_t i = std::distance(costmap.data.rbegin(), iterator);
    data(i) = *iterator != 255 ? *iterator : NAN;
  }

  gridMap.add(layer, data);
  return true;
}

void GridMapRosConverter::toCostmap(
  const grid_map::GridMap & gridMap,
  const std::string & layer, float dataMin, float dataMax,
  nav2_msgs::msg::Costmap & costmap)
{
  costmap.header.frame_id = gridMap.getFrameId();
  costmap.header.stamp = rclcpp::Time(gridMap.getTimestamp());
  // Same as header stamp as we do not load the map.
  costmap.metadata.map_load_time = costmap.header.stamp;
  costmap.metadata.resolution = gridMap.getResolution();
  costmap.metadata.size_x = gridMap.getSize()(0);
  costmap.metadata.size_y = gridMap.getSize()(1);
  Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  costmap.metadata.origin.position.x = position.x();
  costmap.metadata.origin.position.y = position.y();
  costmap.metadata.origin.position.z = 0.0;
  costmap.metadata.origin.orientation.x = 0.0;
  costmap.metadata.origin.orientation.y = 0.0;
  costmap.metadata.origin.orientation.z = 0.0;
  costmap.metadata.origin.orientation.w = 1.0;
  size_t nCells = gridMap.getSize().prod();
  costmap.data.resize(nCells);

  // Costmap cell values are in the range [0,254], 255 is unknown space.
  const float cellMin = 0;
  const float cellMax = 254;
  const float cellRange = cellMax - cellMin;

  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    float value = (gridMap.at(layer, *iterator) - dataMin) / (dataMax - dataMin);
    if (isnan(value)) {
      value = 255;
    } else {
      value = cellMin + min(max(0.0f, value), 1.0f) * cellRange;
    }
    size_t index = getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
    // Reverse cell order because of different conventions between Costmap and grid map.
    costmap.data[nCells - index - 1] = value;
  }
}

void GridMapRosConverter::toGridCells(
  const grid_map::GridMap & gridMap, const std::string & layer,
  float lowerThreshold, float upperThreshold,
  nav_msgs::msg::GridCells & gridCells)
{
  gridCells.header.frame_id = gridMap.getFrameId();
  gridCells.header.stamp = rclcpp::Time(gridMap.getTimestamp());
  gridCells.cell_width = gridMap.getResolution();
  gridCells.cell_height = gridMap.getResolution();

  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    if (!gridMap.isValid(*iterator, layer)) {continue;}
    if (gridMap.at(layer, *iterator) >= lowerThreshold &&
      gridMap.at(layer, *iterator) <= upperThreshold)
    {
      Position position;
      gridMap.getPosition(*iterator, position);
      geometry_msgs::msg::Point point;
      point.x = position.x();
      point.y = position.y();
      gridCells.cells.push_back(point);
    }
  }
}

bool GridMapRosConverter::initializeFromImage(
  const sensor_msgs::msg::Image & image,
  const double resolution, grid_map::GridMap & gridMap,
  const grid_map::Position & position)
{
  const double lengthX = resolution * image.height;
  const double lengthY = resolution * image.width;
  Length length(lengthX, lengthY);
  gridMap.setGeometry(length, resolution, position);
  gridMap.setFrameId(image.header.frame_id);
  gridMap.setTimestamp(rclcpp::Time(image.header.stamp).nanoseconds());
  return true;
}

bool GridMapRosConverter::addLayerFromImage(
  const sensor_msgs::msg::Image & image,
  const std::string & layer, grid_map::GridMap & gridMap,
  const float lowerValue, const float upperValue,
  const double alphaThreshold)
{
  cv_bridge::CvImageConstPtr cvImage;
  try {
    // TODO(needs_assignment) Use `toCvShared()`?
    cvImage = cv_bridge::toCvCopy(image, image.encoding);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("addLayerFromImage"), "cv_bridge exception: %s", e.what());
    return false;
  }

  const int cvEncoding = cv_bridge::getCvType(image.encoding);
  switch (cvEncoding) {
    case CV_8UC1:
      return GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
        cvImage->image, layer, gridMap,
        lowerValue, upperValue,
        alphaThreshold);
    case CV_8UC3:
      return GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
        cvImage->image, layer, gridMap,
        lowerValue, upperValue,
        alphaThreshold);
    case CV_8UC4:
      return GridMapCvConverter::addLayerFromImage<unsigned char, 4>(
        cvImage->image, layer, gridMap,
        lowerValue, upperValue,
        alphaThreshold);
    case CV_16UC1:
      return GridMapCvConverter::addLayerFromImage<uint16_t, 1>(
        cvImage->image, layer,
        gridMap, lowerValue,
        upperValue, alphaThreshold);
    case CV_16UC3:
      return GridMapCvConverter::addLayerFromImage<uint16_t, 3>(
        cvImage->image, layer,
        gridMap, lowerValue,
        upperValue, alphaThreshold);
    case CV_16UC4:
      return GridMapCvConverter::addLayerFromImage<uint16_t, 4>(
        cvImage->image, layer,
        gridMap, lowerValue,
        upperValue, alphaThreshold);
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "addLayerFromImage"),
        "Expected MONO8, MONO16, RGB(A)8, RGB(A)16, BGR(A)8, or BGR(A)16 image encoding.");
      return false;
  }
}

bool GridMapRosConverter::addColorLayerFromImage(
  const sensor_msgs::msg::Image & image,
  const std::string & layer,
  grid_map::GridMap & gridMap)
{
  cv_bridge::CvImageConstPtr cvImage;
  try {
    cvImage = cv_bridge::toCvCopy(image, image.encoding);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("loadFromBag"), "cv_bridge exception: %s", e.what());
    return false;
  }

  const int cvEncoding = cv_bridge::getCvType(image.encoding);
  switch (cvEncoding) {
    case CV_8UC3:
      return GridMapCvConverter::addColorLayerFromImage<unsigned char, 3>(
        cvImage->image, layer,
        gridMap);
    case CV_8UC4:
      return GridMapCvConverter::addColorLayerFromImage<unsigned char, 4>(
        cvImage->image, layer,
        gridMap);
    case CV_16UC3:
      return GridMapCvConverter::addColorLayerFromImage<uint16_t, 3>(
        cvImage->image, layer,
        gridMap);
    case CV_16UC4:
      return GridMapCvConverter::addColorLayerFromImage<uint16_t, 4>(
        cvImage->image, layer,
        gridMap);
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "addColorLayerFromImage"),
        "Expected RGB(A)8, RGB(A)16, BGR(A)8, or BGR(A)16 image encoding.");
      return false;
  }
}

bool GridMapRosConverter::toImage(
  const grid_map::GridMap & gridMap, const std::string & layer,
  const std::string encoding, sensor_msgs::msg::Image & image)
{
  cv_bridge::CvImage cvImage;
  if (!toCvImage(gridMap, layer, encoding, cvImage)) {return false;}
  cvImage.toImageMsg(image);
  return true;
}

bool GridMapRosConverter::toImage(
  const grid_map::GridMap & gridMap, const std::string & layer,
  const std::string encoding, const float lowerValue,
  const float upperValue, sensor_msgs::msg::Image & image)
{
  cv_bridge::CvImage cvImage;
  if (!toCvImage(gridMap, layer, encoding, lowerValue, upperValue, cvImage)) {return false;}
  cvImage.toImageMsg(image);
  return true;
}

bool GridMapRosConverter::toCvImage(
  const grid_map::GridMap & gridMap, const std::string & layer,
  const std::string encoding, cv_bridge::CvImage & cvImage)
{
  const float minValue = gridMap.get(layer).minCoeffOfFinites();
  const float maxValue = gridMap.get(layer).maxCoeffOfFinites();
  return toCvImage(gridMap, layer, encoding, minValue, maxValue, cvImage);
}

bool GridMapRosConverter::toCvImage(
  const grid_map::GridMap & gridMap, const std::string & layer,
  const std::string encoding, const float lowerValue,
  const float upperValue, cv_bridge::CvImage & cvImage)
{
  cvImage.header.stamp = rclcpp::Time(gridMap.getTimestamp());
  cvImage.header.frame_id = gridMap.getFrameId();
  cvImage.encoding = encoding;

  const int cvEncoding = cv_bridge::getCvType(encoding);
  switch (cvEncoding) {
    case CV_8UC1:
      return GridMapCvConverter::toImage<unsigned char, 1>(
        gridMap, layer, cvEncoding, lowerValue,
        upperValue, cvImage.image);
    case CV_8UC3:
      return GridMapCvConverter::toImage<unsigned char, 3>(
        gridMap, layer, cvEncoding, lowerValue,
        upperValue, cvImage.image);
    case CV_8UC4:
      return GridMapCvConverter::toImage<unsigned char, 4>(
        gridMap, layer, cvEncoding, lowerValue,
        upperValue, cvImage.image);
    case CV_16UC1:
      return GridMapCvConverter::toImage<uint16_t, 1>(
        gridMap, layer, cvEncoding, lowerValue,
        upperValue, cvImage.image);
    case CV_16UC3:
      return GridMapCvConverter::toImage<uint16_t, 3>(
        gridMap, layer, cvEncoding, lowerValue,
        upperValue, cvImage.image);
    case CV_16UC4:
      return GridMapCvConverter::toImage<uint16_t, 4>(
        gridMap, layer, cvEncoding, lowerValue,
        upperValue, cvImage.image);
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "toCvImage"),
        "Expected MONO8, MONO16, RGB(A)8, RGB(A)16, BGR(A)8, or BGR(A)16 image encoding.");
      return false;
  }
}

bool GridMapRosConverter::saveToBag(
  const grid_map::GridMap & gridMap, const std::string & pathToBag,
  const std::string & topic)
{
  auto message = toMessage(gridMap);

  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<grid_map_msgs::msg::GridMap> serialization;
  serialization.serialize_message(message.get(), &serialized_msg);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = pathToBag;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Writer writer(std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
  writer.open(storage_options, converter_options);

  rosbag2_storage::TopicMetadata tm;
  tm.name = topic;
  tm.type = "grid_map_msgs/msg/GridMap";
  tm.serialization_format = "cdr";
  writer.create_topic(tm);

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

  auto ret = rcutils_system_time_now(&bag_message->time_stamp);
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("saveToBag"), "couldn't assign time rosbag message");
  }

  bag_message->topic_name = tm.name;
  bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    &serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});

  writer.write(bag_message);
  return true;
}

bool GridMapRosConverter::loadFromBag(
  const std::string & pathToBag, const std::string & topic,
  grid_map::GridMap & gridMap)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = pathToBag;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  reader.open(storage_options, converter_options);

  bool isDataFound = false;

  grid_map_msgs::msg::GridMap extracted_gridmap_msg;
  rclcpp::Serialization<grid_map_msgs::msg::GridMap> serialization;

  // Validate the received bag topic exists and
  // is of the correct type to prevent later serialization exception.
  const auto topic_metadata = reader.get_all_topics_and_types();
  bool topic_is_correct_type = false;
  for (const auto & m : topic_metadata) {
    if (m.name == topic && m.type == "grid_map_msgs/msg/GridMap") {
      topic_is_correct_type = true;
    }
  }
  if (!topic_is_correct_type) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "loadFromBag"), "Bagfile does not contain a GridMap message on the expected topic '%s'",
      topic.c_str());
    return false;
  }

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (bag_message == nullptr) {
      continue;
    }

    // Only read messages on the correct topic.
    // https://github.com/ANYbotics/grid_map/issues/401
    if (bag_message->topic_name != topic) {
      continue;
    }

    if (bag_message->serialized_data != NULL) {
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(&extracted_serialized_msg, &extracted_gridmap_msg);
      fromMessage(extracted_gridmap_msg, gridMap);
      isDataFound = true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("loadFromBag"), "Unable to load data from ROS bag.");
      return false;
    }
  }
  if (!isDataFound) {
    RCLCPP_WARN(
      rclcpp::get_logger(
        "loadFromBag"), "No data under the topic %s was found.", topic.c_str());
  }
  return true;
}
}  // namespace grid_map
