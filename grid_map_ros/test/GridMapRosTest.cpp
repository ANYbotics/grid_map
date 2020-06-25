/*
 * GridMapRosTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Peter Fankhauser, Martin Wermelinger
 *	 Institute: ETH Zurich, ANYbotics
 */

// gtest
#include <gtest/gtest.h>
#include <stdlib.h>

// Eigen
#include <Eigen/Core>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/image_encodings.hpp>

// STD
#include <string>
#include <vector>
#include <iterator>
#include <limits>
#include <chrono>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/gtest_eigen.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

using namespace std;  // NOLINT
using namespace grid_map;  // NOLINT

TEST(RosMessageConversion, roundTrip)
{
  GridMap mapIn({"layer"});
  mapIn.setGeometry(Length(2.0, 3.0), 0.5, Position(1.0, 1.5));
  mapIn["layer"].setRandom();

  grid_map_msgs::msg::GridMap message;
  GridMapRosConverter::toMessage(mapIn, message);
  GridMap mapOut;
  GridMapRosConverter::fromMessage(message, mapOut);

  for (size_t i = 0; i < mapIn.getLayers().size(); ++i) {
    EXPECT_EQ(mapIn.getLayers().at(i), mapOut.getLayers().at(i));
    const std::string layer = mapIn.getLayers().at(i);
    EXPECT_TRUE((mapIn[layer].array() == mapOut[layer].array()).all());
  }

  EXPECT_EQ(mapIn.getFrameId(), mapOut.getFrameId());
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getPosition().array() == mapOut.getPosition().array()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(RosbagHandling, saveLoad)
{
  string layer = "layer";
  string pathToBag = "test.bag";
  string topic = "topic";
  vector<string> layers;
  layers.push_back(layer);
  grid_map::GridMap gridMapIn(layers), gridMapOut;
  gridMapIn.setGeometry(grid_map::Length(1.0, 1.0), 0.5);
  double a = 1.0;
  for (grid_map::GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    gridMapIn.at(layer, *iterator) = a;
    a += 1.0;
  }

  EXPECT_FALSE(gridMapOut.exists(layer));

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    EXPECT_DOUBLE_EQ(gridMapIn.at(layer, *iterator), gridMapOut.at(layer, *iterator));
  }
}

TEST(RosbagHandling, saveLoadWithTime)
{
  string layer = "layer";
  string pathToBag = "test.bag";
  string topic = "topic";
  vector<string> layers;
  layers.push_back(layer);
  GridMap gridMapIn(layers), gridMapOut;
  gridMapIn.setGeometry(grid_map::Length(1.0, 1.0), 0.5);
  double a = 1.0;
  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    gridMapIn.at(layer, *iterator) = a;
    a += 1.0;
  }

  EXPECT_FALSE(gridMapOut.exists(layer));

  // TODO(needs_assignment) Do other time than now.
  rclcpp::Clock clock;
  gridMapIn.setTimestamp(clock.now().nanoseconds());

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    EXPECT_DOUBLE_EQ(gridMapIn.at(layer, *iterator), gridMapOut.at(layer, *iterator));
  }
}

TEST(OccupancyGridConversion, withMove)
{
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(8.0, 5.0), 0.5, grid_map::Position(0.0, 0.0));
  map.add("layer", 1.0);

  // Convert to OccupancyGrid msg.
  nav2_msgs::msg::Costmap costmap;
  GridMapRosConverter::toCostmap(map, "layer", 0.0, 1.0, costmap);

  // Expect the (0, 0) cell to have value 100.
  EXPECT_DOUBLE_EQ(100.0, costmap.data[0]);

  // Move the map, so the cell (0, 0) will move to unobserved space.
  map.move(grid_map::Position(-1.0, -1.0));

  // Convert again to OccupancyGrid msg.
  nav2_msgs::msg::Costmap costmapNew;
  GridMapRosConverter::toCostmap(map, "layer", 0.0, 1.0, costmapNew);

  // Now the (0, 0) cell should be unobserved (-1).
  EXPECT_DOUBLE_EQ(-1.0, costmapNew.data[0]);
}

TEST(OccupancyGridConversion, roundTrip)
{
  // Create occupancy grid.
  nav2_msgs::msg::Costmap costmap;
  costmap.header.stamp = rclcpp::Time(5.0);
  costmap.header.frame_id = "map";
  costmap.metadata.resolution = 0.1;
  costmap.metadata.size_x = 50;
  costmap.metadata.size_y = 100;
  costmap.metadata.origin.position.x = 3.0;
  costmap.metadata.origin.position.y = 6.0;
  costmap.metadata.origin.orientation.w = 1.0;
  costmap.data.resize(costmap.metadata.size_x * costmap.metadata.size_y);

  for (auto & cell : costmap.data) {
    cell = rand_r(static_cast<‘unsigned int*>(time(0))) % 102 - 1;  // [-1, 100]
  }

  // Convert to grid map.
  GridMap gridMap;
  GridMapRosConverter::fromCostmap(costmap, "layer", gridMap);

  // Convert back to occupancy grid.
  nav2_msgs::msg::Costmap costmapResult;
  GridMapRosConverter::toCostmap(gridMap, "layer", -1.0, 100.0, costmapResult);

  // Check map info.
  EXPECT_EQ(costmap.header.stamp, costmapResult.header.stamp);
  EXPECT_EQ(costmap.header.frame_id, costmapResult.header.frame_id);
  EXPECT_EQ(costmap.metadata.size_x, costmapResult.metadata.size_x);
  EXPECT_EQ(costmap.metadata.size_y, costmapResult.metadata.size_y);
  EXPECT_DOUBLE_EQ(
    costmap.metadata.origin.position.x,
    costmapResult.metadata.origin.position.x);
  EXPECT_DOUBLE_EQ(
    costmap.metadata.origin.position.x,
    costmapResult.metadata.origin.position.x);
  EXPECT_DOUBLE_EQ(
    costmap.metadata.origin.orientation.x,
    costmapResult.metadata.origin.orientation.x);
  EXPECT_DOUBLE_EQ(
    costmap.metadata.origin.orientation.y,
    costmapResult.metadata.origin.orientation.y);
  EXPECT_DOUBLE_EQ(
    costmap.metadata.origin.orientation.z,
    costmapResult.metadata.origin.orientation.z);
  EXPECT_DOUBLE_EQ(
    costmap.metadata.origin.orientation.w,
    costmapResult.metadata.origin.orientation.w);

  // Check map data.
  for (std::vector<uint8_t>::iterator iterator = costmap.data.begin();
    iterator != costmap.data.end(); ++iterator)
  {
    size_t i = std::distance(costmap.data.begin(), iterator);
    EXPECT_EQ(static_cast<int>(*iterator), static_cast<int>(costmapResult.data[i]));
  }
}

TEST(ImageConversion, roundTripBGRA8)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn["layer"].setRandom();
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image message.
  sensor_msgs::msg::Image image;
  GridMapRosConverter::toImage(
    mapIn, "layer", sensor_msgs::image_encodings::BGRA8, minValue,
    maxValue, image);

  // Convert back to grid map.
  GridMap mapOut;
  GridMapRosConverter::initializeFromImage(image, mapIn.getResolution(), mapOut);
  GridMapRosConverter::addLayerFromImage(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) /
    static_cast<float>(std::numeric_limits<unsigned char>::max());
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTripMONO16)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn["layer"].setRandom();
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image message.
  sensor_msgs::msg::Image image;
  GridMapRosConverter::toImage(
    mapIn, "layer", sensor_msgs::image_encodings::MONO16,
    minValue, maxValue, image);

  // Convert back to grid map.
  GridMap mapOut;
  GridMapRosConverter::initializeFromImage(image, mapIn.getResolution(), mapOut);
  GridMapRosConverter::addLayerFromImage(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  // TODO(needs_assignment) Why is factor 300 necessary?
  const float resolution = 300.0 * (maxValue - minValue) /
    static_cast<float>(std::numeric_limits<uint16_t>::max());
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_EQ(mapIn.getTimestamp(), mapOut.getTimestamp());
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}
