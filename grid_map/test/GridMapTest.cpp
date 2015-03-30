/*
 * GridMapTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map/GridMapRosConverter.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

// STD
#include <string>

using namespace std;
using namespace grid_map;

TEST(RosbagHandling, saveLoad)
{
  Eigen::Matrix2f dataIn, dataOut;
  dataIn << 1.0, 2.0,
            3.0, 4.0;
  string layer = "layer";
  string pathToBag = "test.bag";
  string topic = "topic";
  grid_map::GridMap gridMapIn, gridMapOut;
  gridMapIn.add(layer, dataIn);

  EXPECT_FALSE(gridMapOut.exists(layer));

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  dataOut = gridMapOut.get(layer);

  // TODO Do full matrix comparison.
  EXPECT_DOUBLE_EQ(dataIn(0,0), dataOut(0,0));
  EXPECT_DOUBLE_EQ(dataIn(1,1), dataOut(1,1));
}

TEST(RosbagHandling, saveLoadWithTime)
{
  Eigen::Matrix2f dataIn, dataOut;
  dataIn << 1.0, 2.0,
            3.0, 4.0;
  string layer = "layer";
  string pathToBag = "test.bag";
  string topic = "topic";
  grid_map::GridMap gridMapIn, gridMapOut;
  gridMapIn.add(layer, dataIn);

  EXPECT_FALSE(gridMapOut.exists(layer));

  if (!ros::Time::isValid()) ros::Time::init();
  // TODO Do other time than now.
  gridMapIn.setTimestamp(ros::Time::now().toNSec());

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  dataOut = gridMapOut.get(layer);

  // TODO Do full matrix comparison.
  EXPECT_DOUBLE_EQ(dataIn(0,0), dataOut(0,0));
  EXPECT_DOUBLE_EQ(dataIn(1,1), dataOut(1,1));
}
