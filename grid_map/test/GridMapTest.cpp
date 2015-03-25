/*
 * GridMapTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace grid_map_lib;
using namespace grid_map;

TEST(RosbagHandling, saveLoad)
{
  Matrix2f dataIn, dataOut;
  dataIn << 1.0, 2.0,
            3.0, 4.0;
  string type = "type";
  string bagName = "bagName";
  string topicName = "topicName";
  grid_map::GridMap gridMapIn, gridMapOut;
  gridMapIn.add(type, dataIn);

  EXPECT_FALSE(gridMapOut.exists(type));

  EXPECT_TRUE(gridMapIn.toRosbag(bagName, topicName));
  EXPECT_TRUE(gridMapOut.fromRosbag(bagName, topicName));

  EXPECT_TRUE(gridMapOut.exists(type));

  dataOut = gridMapOut.get(type);

  EXPECT_DOUBLE_EQ(dataIn(0,0), dataOut(0,0));
  EXPECT_DOUBLE_EQ(dataIn(1,1), dataOut(1,1));

}
