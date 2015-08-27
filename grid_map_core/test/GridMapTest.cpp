/*
 * GridMapTest.cpp
 *
 *  Created on: Aug 26, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

// Math
#include <math.h>

using namespace std;
using namespace grid_map;

TEST(GridMap, Move)
{
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer", 0.0);
  map.setBasicLayers(map.getLayers());
  std::vector<BufferRegion> regions;
  map.move(Position(-3.0, -2.0), regions);
  Index startIndex = map.getStartIndex();

  EXPECT_EQ(3, startIndex(0));
  EXPECT_EQ(2, startIndex(1));

  EXPECT_FALSE(map.isValid(Index(0, 0))); // TODO Check entire map.
  EXPECT_TRUE(map.isValid(Index(3, 2)));
  EXPECT_FALSE(map.isValid(Index(2, 2)));
  EXPECT_FALSE(map.isValid(Index(3, 1)));
  EXPECT_TRUE(map.isValid(Index(7, 4)));

  EXPECT_EQ(2, regions.size());
  EXPECT_EQ(0, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(3, regions[0].getSize()[0]);
  EXPECT_EQ(5, regions[0].getSize()[1]);
  EXPECT_EQ(0, regions[1].getStartIndex()[0]);
  EXPECT_EQ(0, regions[1].getStartIndex()[1]);
  EXPECT_EQ(8, regions[1].getSize()[0]);
  EXPECT_EQ(2, regions[1].getSize()[1]);
}
