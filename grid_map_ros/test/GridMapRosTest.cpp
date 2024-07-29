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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rcpputils/filesystem_helper.hpp>

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

  auto message = GridMapRosConverter::toMessage(mapIn);
  GridMap mapOut;
  GridMapRosConverter::fromMessage(*message, mapOut);

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

  // Cleaning in case the previous bag was not removed
  rcpputils::fs::path dir(pathToBag);
  EXPECT_TRUE(!dir.exists() || rcpputils::fs::remove_all(dir));
  EXPECT_TRUE(!dir.exists() || rcpputils::fs::create_directories(dir));

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));
  EXPECT_TRUE(gridMapOut.exists(layer));

  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    EXPECT_DOUBLE_EQ(gridMapIn.at(layer, *iterator), gridMapOut.at(layer, *iterator));
  }

  // Removing the created bag
  rcpputils::fs::remove_all(dir);
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

  // TODO(needs_assignment): Do other time than now.
  rclcpp::Clock clock;
  gridMapIn.setTimestamp(clock.now().nanoseconds());

  // Cleaning in case the previous bag was not removed
  rcpputils::fs::path dir(pathToBag);
  EXPECT_TRUE(!dir.exists() || rcpputils::fs::remove_all(dir));
  EXPECT_TRUE(!dir.exists() || rcpputils::fs::create_directories(dir));

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    EXPECT_DOUBLE_EQ(gridMapIn.at(layer, *iterator), gridMapOut.at(layer, *iterator));
  }

  // Removing the created bag
  rcpputils::fs::remove_all(dir);
}

TEST(RosbagHandling, wrongTopicType)
{
  // This test validates LoadFromBag will reject a topic of the wrong type.
  // See https://github.com/ANYbotics/grid_map/issues/401.

  std::string pathToBag = "double_chatter";
  string topic = "/chatter1";
  GridMap gridMapOut;
  rcpputils::fs::path dir(pathToBag);

  EXPECT_FALSE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));
}

TEST(RosbagHandling, checkTopicTypes)
{
  // This test validates loadFromBag will reject a topic of the wrong type or
  // non-existing topic and accept a correct topic.

  std::string pathToBag = "test_multitopic";
  string topic_wrong = "/chatter";
  string topic_correct = "/grid_map";
  string topic_non_existing = "/grid_map_non_existing";
  GridMap gridMapOut;
  rcpputils::fs::path dir(pathToBag);

  EXPECT_FALSE(GridMapRosConverter::loadFromBag(pathToBag, topic_wrong, gridMapOut));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic_correct, gridMapOut));
  EXPECT_FALSE(GridMapRosConverter::loadFromBag(pathToBag, topic_non_existing, gridMapOut));
}

TEST(OccupancyGridConversion, withMove)
{
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(8.0, 5.0), 0.5, grid_map::Position(0.0, 0.0));
  map.add("layer", 1.0);

  // Convert to OccupancyGrid msg.
  nav_msgs::msg::OccupancyGrid occupancyGrid;
  GridMapRosConverter::toOccupancyGrid(map, "layer", 0.0, 1.0, occupancyGrid);

  // Expect the (0, 0) cell to have value 100.
  EXPECT_DOUBLE_EQ(100.0, occupancyGrid.data[0]);

  // Move the map, so the cell (0, 0) will move to unobserved space.
  map.move(grid_map::Position(-1.0, -1.0));

  // Convert again to OccupancyGrid msg.
  nav_msgs::msg::OccupancyGrid occupancyGridNew;
  GridMapRosConverter::toOccupancyGrid(map, "layer", 0.0, 1.0, occupancyGridNew);

  // Now the (0, 0) cell should be unobserved (-1).
  EXPECT_DOUBLE_EQ(-1.0, occupancyGridNew.data[0]);
}

TEST(OccupancyGridConversion, roundTrip)
{
  // Create occupancy grid.
  nav_msgs::msg::OccupancyGrid occupancyGrid;
  occupancyGrid.header.stamp = rclcpp::Time(5.0);
  occupancyGrid.header.frame_id = "map";
  occupancyGrid.info.resolution = 0.1;
  occupancyGrid.info.width = 50;
  occupancyGrid.info.height = 100;
  occupancyGrid.info.origin.position.x = 3.0;
  occupancyGrid.info.origin.position.y = 6.0;
  occupancyGrid.info.origin.orientation.w = 1.0;
  occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

  unsigned int seed = time(0);
  for (auto & cell : occupancyGrid.data) {
    cell = rand_r(&seed) % 102 - 1;  // [-1, 100]
  }

  // Convert to grid map.
  GridMap gridMap;
  GridMapRosConverter::fromOccupancyGrid(occupancyGrid, "layer", gridMap);

  // Convert back to occupancy grid.
  nav_msgs::msg::OccupancyGrid occupancyGridResult;
  GridMapRosConverter::toOccupancyGrid(gridMap, "layer", -1.0, 100.0, occupancyGridResult);

  // Check map info.
  EXPECT_EQ(occupancyGrid.header.stamp, occupancyGridResult.header.stamp);
  EXPECT_EQ(occupancyGrid.header.frame_id, occupancyGridResult.header.frame_id);
  EXPECT_EQ(occupancyGrid.info.width, occupancyGridResult.info.width);
  EXPECT_EQ(occupancyGrid.info.height, occupancyGridResult.info.height);
  EXPECT_DOUBLE_EQ(
    occupancyGrid.info.origin.position.x,
    occupancyGridResult.info.origin.position.x);
  EXPECT_DOUBLE_EQ(
    occupancyGrid.info.origin.position.x,
    occupancyGridResult.info.origin.position.x);
  EXPECT_DOUBLE_EQ(
    occupancyGrid.info.origin.orientation.x,
    occupancyGridResult.info.origin.orientation.x);
  EXPECT_DOUBLE_EQ(
    occupancyGrid.info.origin.orientation.y,
    occupancyGridResult.info.origin.orientation.y);
  EXPECT_DOUBLE_EQ(
    occupancyGrid.info.origin.orientation.z,
    occupancyGridResult.info.origin.orientation.z);
  EXPECT_DOUBLE_EQ(
    occupancyGrid.info.origin.orientation.w,
    occupancyGridResult.info.origin.orientation.w);

  // Check map data.
  for (std::vector<int8_t>::iterator iterator = occupancyGrid.data.begin();
    iterator != occupancyGrid.data.end(); ++iterator)
  {
    size_t i = std::distance(occupancyGrid.data.begin(), iterator);
    EXPECT_EQ(static_cast<int>(*iterator), static_cast<int>(occupancyGridResult.data[i]));
  }
}

TEST(CostmapConversion, withMove)
{
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(8.0, 5.0), 0.5, grid_map::Position(0.0, 0.0));
  map.add("layer", 1.0);

  // Convert to Costmap msg.
  nav2_msgs::msg::Costmap costmap;
  GridMapRosConverter::toCostmap(map, "layer", 0.0, 1.0, costmap);

  // Expect the (0, 0) cell to have value 254.
  EXPECT_EQ(254, costmap.data[0]);

  // Move the map, so the cell (0, 0) will move to unobserved space.
  map.move(grid_map::Position(-1.0, -1.0));

  // Convert again to Costmap msg.
  nav2_msgs::msg::Costmap costmapNew;
  GridMapRosConverter::toCostmap(map, "layer", 0.0, 1.0, costmapNew);

  // Now the (0, 0) cell should be unobserved (255).
  EXPECT_EQ(255, costmapNew.data[0]);
}

TEST(CostmapConversion, roundTrip)
{
  // Create Costmap grid.
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

  unsigned int seed = time(0);
  for (auto & cell : costmap.data) {
    cell = rand_r(&seed) % 256;  // [0, 255]
  }

  // Convert to grid map.
  GridMap gridMap;
  GridMapRosConverter::fromCostmap(costmap, "layer", gridMap);

  // Convert back to costmap.
  nav2_msgs::msg::Costmap costmapResult;
  GridMapRosConverter::toCostmap(gridMap, "layer", 0, 254.0, costmapResult);

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
    EXPECT_EQ(*iterator, costmapResult.data[i]);
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
