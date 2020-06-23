/*
 * LineIteratorTest.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: Dominic Jud
 *	 Institute: ETH Zurich, ANYbotics
 */

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

#include "grid_map_core/iterators/LineIterator.hpp"
#include "grid_map_core/GridMap.hpp"


TEST(LineIterator, StartOutsideMap)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(
    grid_map::LineIterator iterator(
      map, grid_map::Position(2.0, 2.0),
      grid_map::Position(0.0, 0.0)));
  grid_map::LineIterator iterator(map, grid_map::Position(2.0, 2.0), grid_map::Position(0.0, 0.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, EndOutsideMap)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(
    grid_map::LineIterator iterator(
      map, grid_map::Position(0.0, 0.0),
      grid_map::Position(9.0, 6.0)));
  grid_map::LineIterator iterator(map, grid_map::Position(0.0, 0.0), grid_map::Position(9.0, 6.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMap)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(
    grid_map::LineIterator iterator(
      map, grid_map::Position(-7.0, -9.0),
      grid_map::Position(8.0, 8.0)));
  grid_map::LineIterator iterator(map, grid_map::Position(-7.0, -9.0),
    grid_map::Position(8.0, 8.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  ++iterator;
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMapWithoutIntersectingMap)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_THROW(
    grid_map::LineIterator iterator(
      map, grid_map::Position(-8.0, 8.0), grid_map::Position(
        8.0,
        8.0)),
    std::invalid_argument);
}

TEST(LineIterator, MovedMap)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(7.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));
  map.move(grid_map::Position(2.0, 2.0));

  EXPECT_NO_THROW(
    grid_map::LineIterator iterator(
      map, grid_map::Position(0.0, 0.0),
      grid_map::Position(2.0, 2.0)));
  grid_map::LineIterator iterator(map, grid_map::Position(0.0, 0.0), grid_map::Position(2.0, 2.0));
  grid_map::Position point;

  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(0, point.x());
  EXPECT_EQ(0, point.y());

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(1, point.x());
  EXPECT_EQ(1, point.y());

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(2, point.x());
  EXPECT_EQ(2, point.y());

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMovedMap)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(7.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));
  map.move(grid_map::Position(2.0, 2.0));

  EXPECT_NO_THROW(
    grid_map::LineIterator iterator(
      map, grid_map::Position(0.0, 0.0),
      grid_map::Position(8.0, 8.0)));
  grid_map::LineIterator iterator(map, grid_map::Position(0.0, 0.0), grid_map::Position(8.0, 8.0));
  grid_map::Position point;

  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(0, point.x());
  EXPECT_EQ(0, point.y());

  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(1, point.x());
  EXPECT_EQ(1, point.y());
  //
  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(2, point.x());
  EXPECT_EQ(2, point.y());
  //
  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(3, point.x());
  EXPECT_EQ(3, point.y());

  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(4, point.x());
  EXPECT_EQ(4, point.y());

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}
