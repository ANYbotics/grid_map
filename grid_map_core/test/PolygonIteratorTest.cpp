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


TEST(PolygonIterator, FullCover)
{
  std::vector<std::string> types;
  types.push_back("type");
  grid_map::GridMap map(types);
  // bufferSize(8, 5)
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  grid_map::Polygon polygon;
  polygon.addVertex(grid_map::Position(-100.0, 100.0));
  polygon.addVertex(grid_map::Position(100.0, 100.0));
  polygon.addVertex(grid_map::Position(100.0, -100.0));
  polygon.addVertex(grid_map::Position(-100.0, -100.0));

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

TEST(PolygonIterator, Outside)
{
  grid_map::GridMap map({"types"});
  // bufferSize(8, 5)
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  grid_map::Polygon polygon;
  polygon.addVertex(grid_map::Position(99.0, 101.0));
  polygon.addVertex(grid_map::Position(101.0, 101.0));
  polygon.addVertex(grid_map::Position(101.0, 99.0));
  polygon.addVertex(grid_map::Position(99.0, 99.0));

  grid_map::PolygonIterator iterator(map, polygon);

  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Square)
{
  grid_map::GridMap map({"types"});
  // bufferSize(8, 5)
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  grid_map::Polygon polygon;
  polygon.addVertex(grid_map::Position(-1.0, 1.5));
  polygon.addVertex(grid_map::Position(1.0, 1.5));
  polygon.addVertex(grid_map::Position(1.0, -1.5));
  polygon.addVertex(grid_map::Position(-1.0, -1.5));

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

TEST(PolygonIterator, TopLeftTriangle)
{
  grid_map::GridMap map({"types"});
  // bufferSize(8, 5)
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  grid_map::Polygon polygon;
  polygon.addVertex(grid_map::Position(-40.1, 20.6));
  polygon.addVertex(grid_map::Position(40.1, 20.4));
  polygon.addVertex(grid_map::Position(-40.1, -20.6));

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

TEST(PolygonIterator, MoveMap)
{
  grid_map::GridMap map({"layer"});
  // bufferSize(8, 5)
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));
  map.move(grid_map::Position(2.0, 0.0));

  grid_map::Polygon polygon;
  polygon.addVertex(grid_map::Position(6.1, 1.6));
  polygon.addVertex(grid_map::Position(0.9, 1.6));
  polygon.addVertex(grid_map::Position(0.9, -1.6));
  polygon.addVertex(grid_map::Position(6.1, -1.6));
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
