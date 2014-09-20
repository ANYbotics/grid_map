/*
 * PolygonIteratorTest.cpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/PolygonIterator.hpp"
#include "grid_map_lib/GridMap.hpp"

// Eigen
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>

using namespace std;
using namespace Eigen;
using namespace grid_map_lib;

TEST(checkPolygonIterator, FullCover)
{
  vector<string> types;
  types.push_back("type");
  grid_map_lib::GridMap map(types);
  map.setGeometry(Array2d(8.0, 5.0), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)

  grid_map_lib::PolygonIterator::Polygon polygon;
  polygon.push_back(Vector2d(-100.0, 100.0));
  polygon.push_back(Vector2d(100.0, 100.0));
  polygon.push_back(Vector2d(100.0, -100.0));
  polygon.push_back(Vector2d(-100.0, -100.0));

  PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 37; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPassedEnd());
}

TEST(checkPolygonIterator, Outside)
{
  vector<string> types;
  types.push_back("type");
  grid_map_lib::GridMap map(types);
  map.setGeometry(Array2d(8.0, 5.0), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)

  grid_map_lib::PolygonIterator::Polygon polygon;
  polygon.push_back(Vector2d(99.0, 101.0));
  polygon.push_back(Vector2d(101.0, 101.0));
  polygon.push_back(Vector2d(101.0, 99.0));
  polygon.push_back(Vector2d(99.0, 99.0));

  PolygonIterator iterator(map, polygon);

  EXPECT_TRUE(iterator.isPassedEnd());
}

TEST(checkPolygonIterator, Square)
{
  vector<string> types;
  types.push_back("type");
  grid_map_lib::GridMap map(types);
  map.setGeometry(Array2d(8.0, 5.0), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)

  grid_map_lib::PolygonIterator::Polygon polygon;
  polygon.push_back(Vector2d(-1.0, 1.5));
  polygon.push_back(Vector2d(1.0, 1.5));
  polygon.push_back(Vector2d(1.0, -1.5));
  polygon.push_back(Vector2d(-1.0, -1.5));

  PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPassedEnd());
}

TEST(checkPolygonIterator, TopLeftTriangle)
{
  vector<string> types;
  types.push_back("type");
  grid_map_lib::GridMap map(types);
  map.setGeometry(Array2d(8.0, 5.0), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)

  grid_map_lib::PolygonIterator::Polygon polygon;
  polygon.push_back(Vector2d(-40.1, 20.6));
  polygon.push_back(Vector2d(40.1, 20.4));
  polygon.push_back(Vector2d(-40.1, -20.6));

  PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPassedEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  // TODO Extend.
}


