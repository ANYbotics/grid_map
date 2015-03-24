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

using namespace std;
using namespace Eigen;
using namespace grid_map_lib;
using namespace grid_map;

TEST(RosbagHandling, saveLoad)
{
  Matrix2f data;
  data << 1, 2,
          3, 4;
  string type;
  grid_map::GridMap gridMap;
  gridMap.add(type, data);
}
