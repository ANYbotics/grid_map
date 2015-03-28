/*
 * GridMapRosConverter.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMapRosConverter.hpp"
#include "grid_map/GridMapMsgHelpers.hpp"
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>

// STL
#include <limits>
#include <algorithm>

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

  message.layers = gridMap.getLayers();
  message.basic_layers = gridMap.getBasicLayers();

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

// Points
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
        ;
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
  Position positionOfOrigin;
  getPositionOfDataStructureOrigin(gridMap.getPosition(), gridMap.getLength(), positionOfOrigin);
  occupancyGrid.info.origin.position.x = positionOfOrigin.x();
  occupancyGrid.info.origin.position.y = positionOfOrigin.y();
  occupancyGrid.info.origin.position.z = 0.0;
  occupancyGrid.info.origin.orientation.x = 0.0;
  occupancyGrid.info.origin.orientation.y = 0.0;
  occupancyGrid.info.origin.orientation.z = 1.0;  // yes, this is correct.
  occupancyGrid.info.origin.orientation.w = 0.0;
  occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

  // Occupancy probabilities are in the range [0,100].  Unknown is -1.
  const float cellMin = 0;
  const float cellMax = 100;
  const float cellRange = cellMax - cellMin;

  for (GridMapIterator iterator(gridMap); !iterator.isPassedEnd(); ++iterator) {
    float value = (gridMap.at(layer, *iterator) - dataMin) / (dataMax - dataMin);
    if (isnan(value))
      value = -1;
    else
      value = cellMin + min(max(0.0f, value), 1.0f) * cellRange;
    // Occupancy grid claims to be row-major order, but it does not seem that way.
    // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html.
    unsigned int index = get1dIndexFrom2dIndex(*iterator, gridMap.getSize(), false);
    occupancyGrid.data[index] = value;
  }
}

void GridMapRosConverter::toGridCells(const grid_map::GridMap& gridMap, const std::string& layer, float lowerThreshold,
                 float upperThreshold, nav_msgs::GridCells& gridCells)
{
  gridCells.header.frame_id = gridMap.getFrameId();
  gridCells.header.stamp.fromNSec(gridMap.getTimestamp());
  gridCells.cell_width = gridMap.getResolution();
  gridCells.cell_height = gridMap.getResolution();

  for (GridMapIterator iterator(gridMap); !iterator.isPassedEnd(); ++iterator) {
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
  
} /* namespace */
