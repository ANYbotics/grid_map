/*
 * PolygonIteratorTest.cpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

// Limits
#include <cfloat>

// Vector
#include <vector>

#include <string>

#include "grid_map_core/iterators/PolygonIterator.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"

// using namespace std;
// using namespace Eigen;
// using namespace grid_map;

TEST(grid_map::PolygonIterator, FullCover)
{
  std::vector<string> types;
  types.push_back("type");
  grid_map::GridMap map(types);
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  grid_map::Polygon polygon;
  polygon.addVertex(Position(-100.0, 100.0));
  polygon.addVertex(Position(100.0, 100.0));
  polygon.addVertex(Position(100.0, -100.0));
  polygon.addVertex(Position(-100.0, -100.0));

  grid_map::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 37; ++i) {
    ++iterator;
  }

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(grid_map::PolygonIterator, Outside)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  grid_map::Polygon polygon;
  polygon.addVertex(Position(99.0, 101.0));
  polygon.addVertex(Position(101.0, 101.0));
  polygon.addVertex(Position(101.0, 99.0));
  polygon.addVertex(Position(99.0, 99.0));

  grid_map::PolygonIterator iterator(map, polygon);

  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(grid_map::PolygonIterator, Square)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  grid_map::Polygon polygon;
  polygon.addVertex(Position(-1.0, 1.5));
  polygon.addVertex(Position(1.0, 1.5));
  polygon.addVertex(Position(1.0, -1.5));
  polygon.addVertex(Position(-1.0, -1.5));

  grid_map::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(grid_map::PolygonIterator, TopLeftTriangle)
{
  ++iterator;
  polygon.addVertex(Position(40.1, 20.4));
  polygon.addVertex(Position(-40.1, -20.6));

  grid_map::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  // TODO(needs_assignment): Extend.
}

TEST(grid_map::PolygonIterator, MoveMap)
{
  grid_map::GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)
  map.move(Position(2.0, 0.0));

  grid_map::Polygon polygon;
  polygon.addVertex(Position(6.1, 1.6));
  polygon.addVertex(Position(0.9, 1.6));
  polygon.addVertex(Position(0.9, -1.6));
  polygon.addVertex(Position(6.1, -1.6));
  grid_map::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 4; ++i) {
    ++iterator;
  }

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  for (int i = 0; i < 8; ++i) {
    ++iterator;
  }

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}
