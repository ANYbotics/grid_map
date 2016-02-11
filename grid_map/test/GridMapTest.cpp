/*
 * GridMapTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map/GridMapRosConverter.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

// STD
#include <string>
#include <vector>

using namespace std;
using namespace grid_map;

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

  for (grid_map::GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
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
  grid_map::GridMap gridMapIn(layers), gridMapOut;
  gridMapIn.setGeometry(grid_map::Length(1.0, 1.0), 0.5);
  double a = 1.0;
  for (grid_map::GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    gridMapIn.at(layer, *iterator) = a;
    a += 1.0;
  }

  EXPECT_FALSE(gridMapOut.exists(layer));

  if (!ros::Time::isValid()) ros::Time::init();
  // TODO Do other time than now.
  gridMapIn.setTimestamp(ros::Time::now().toNSec());

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  for (grid_map::GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    EXPECT_DOUBLE_EQ(gridMapIn.at(layer, *iterator), gridMapOut.at(layer, *iterator));
  }
}

TEST(ToOccupancyGrid, toOccupancyGridWithMove)
{
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(8.1, 5.1), 0.5, grid_map::Position(0.0, 0.0));
  map.add("layer", 1.0);

  // convert to OccupancyGrid msg
  nav_msgs::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, "layer", 0.0, 1.0, occupancyGrid);

  // expect the (0, 0) cell to have value 100
  EXPECT_DOUBLE_EQ(100.0, occupancyGrid.data[0]);

  // move the map, so the cell (0, 0) will move to unobserved space
  map.move(grid_map::Position(1.0, 1.0));

  // convert again to OGM
  nav_msgs::OccupancyGrid occupancyGridNew;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, "layer", 0.0, 1.0, occupancyGridNew);

  // Now the (0, 0) cell should be unobserved (-1.0)
  EXPECT_DOUBLE_EQ(-1.0, occupancyGridNew.data[0]);
}
