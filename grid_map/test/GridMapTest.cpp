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
