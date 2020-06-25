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
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/serialization_format_converter_factory.hpp>
#include <pluginlib/class_loader.hpp>

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

void GridMapRosConverter::toMessage(
  const grid_map::GridMap & gridMap,
  grid_map_msgs::msg::GridMap & message)
{
  toMessage(gridMap, gridMap.getLayers(), message);
}

void GridMapRosConverter::toMessage(
  const grid_map::GridMap & gridMap, const std::vector<std::string> & layers,
  grid_map_msgs::msg::GridMap & message)
{
  message.header.stamp = rclcpp::Time(gridMap.getTimestamp());
  message.header.frame_id = gridMap.getFrameId();
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
  for (const auto & layer : layers) {
    std_msgs::msg::Float32MultiArray dataArray;
    matrixEigenCopyToMultiArrayMessage(gridMap.get(layer), dataArray);
    message.data.push_back(dataArray);
  }

  message.outer_start_index = gridMap.getStartIndex()(0);
  message.inner_start_index = gridMap.getStartIndex()(1);
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
      "Orientation of occupancy grid: \n%s", occupancyGrid.info.origin.orientation);
    return false;
  }

  if (size.prod() != occupancyGrid.data.size()) {
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
  uint64_t time = gridMap.getTimestamp();

  if (!time) {
    rclcpp::Clock clock;
    time = clock.now().nanoseconds();
  }

  std::unique_ptr<rosbag2_cpp::SerializationFormatConverterFactory> factory;
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatSerializer> cdr_serializer;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_typesupport_library;
  std::shared_ptr<rcpputils::SharedLibrary> rosidl_typesupport_library;
  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> msg_to_write;
  // std::unique_ptr<MemoryManagement> memory_management;  // Not sure if that is completely needed
  const rosidl_message_type_support_t * rosidl_type_support;
  const rosidl_message_type_support_t * introspection_type_support;
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

  factory = std::make_unique<rosbag2_cpp::SerializationFormatConverterFactory>();
  cdr_serializer = factory->load_serializer("cdr");

  introspection_typesupport_library = rosbag2_cpp::get_typesupport_library(
    "grid_map_msgs/GridMap", "rosidl_typesupport_introspection_cpp");
  introspection_type_support = rosbag2_cpp::get_typesupport_handle(
    "grid_map_msgs/GridMap", "rosidl_typesupport_introspection_cpp",
    introspection_typesupport_library);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  msg_to_write =
    rosbag2_cpp::allocate_introspection_message(introspection_type_support, &allocator);
  rosbag2_cpp::introspection_message_set_topic_name(msg_to_write.get(), topic.c_str());
  // msg_to_write->time_stamp = 1;  // TODO(marwan99): add corect time val

  auto ros_message = reinterpret_cast<grid_map_msgs::msg::GridMap *>(msg_to_write->message);
  toMessage(gridMap, *ros_message);

  rosidl_typesupport_library = rosbag2_cpp::get_typesupport_library(
    "grid_map_msgs/GridMap", "rosidl_typesupport_cpp");
  rosidl_type_support = rosbag2_cpp::get_typesupport_handle(
    "grid_map_msgs/GridMap", "rosidl_typesupport_cpp", rosidl_typesupport_library);

  // memory_management = std::make_unique<MemoryManagement>();
  // serialized_message->serialized_data = memory_management->make_initialized_message();

  cdr_serializer->serialize(msg_to_write, rosidl_type_support, serialized_message);

  rosbag2_cpp::StorageOptions storage_options = rosbag2_cpp::StorageOptions{};
  storage_options.uri = pathToBag;

  std::unique_ptr<rosbag2_cpp::Writer> bag;
  bag->open(storage_options, {"rmw_format", "rmw_format"});
  bag->create_topic({topic, "grid_map_msgs/GridMap", "", ""});
  bag->write(serialized_message);
  return true;
}

bool GridMapRosConverter::loadFromBag(
  const std::string & pathToBag, const std::string & topic,
  grid_map::GridMap & gridMap)
{
  std::unique_ptr<rosbag2_cpp::SerializationFormatConverterFactory> factory;
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
  cdr_deserializer_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_typesupport_library;
  std::shared_ptr<rcpputils::SharedLibrary> rosidl_typesupport_library;
  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> read_msg;
  const rosidl_message_type_support_t * rosidl_type_support;
  const rosidl_message_type_support_t * introspection_type_support;
  auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

  rosidl_typesupport_library = rosbag2_cpp::get_typesupport_library(
    "grid_map_msgs/GridMap", "rosidl_typesupport_cpp");
  rosidl_type_support = rosbag2_cpp::get_typesupport_handle(
    "grid_map_msgs/GridMap", "rosidl_typesupport_cpp", rosidl_typesupport_library);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  read_msg = rosbag2_cpp::allocate_introspection_message(introspection_type_support, &allocator);
  rosbag2_cpp::introspection_message_set_topic_name(read_msg.get(), topic.c_str());

  std::unique_ptr<rosbag2_cpp::Reader> bag;
  rosbag2_cpp::StorageOptions storage_options = rosbag2_cpp::StorageOptions{};
  rosbag2_storage::StorageFilter storage_filter;

  storage_options.uri = pathToBag;
  storage_filter.topics.push_back(topic);

  bag->open(storage_options, {"", "rmw_format"});
  bag->set_filter(storage_filter);

  bool isDataFound = false;
  while (bag->has_next()) {
    serialized_message = bag->read_next();
    cdr_deserializer_->deserialize(serialized_message, rosidl_type_support, read_msg);
    grid_map_msgs::msg::GridMap * message =
      static_cast<grid_map_msgs::msg::GridMap *>(read_msg->message);
    if (message != NULL) {
      fromMessage(*message, gridMap);
      isDataFound = true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("loadFromBag"), "Unable to load data from ROS bag.");
      return false;
    }
  }
  if (!isDataFound) {
    RCLCPP_WARN(rclcpp::get_logger("loadFromBag"), "No data under the topic %s was found.", topic);
  }
  return true;
}
}  // namespace grid_map
