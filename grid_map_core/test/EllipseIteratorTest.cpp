/*
 * EllipseIteratorTest.cpp
 *
 *  Created on: Dec 2, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// Eigen
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>

#include "grid_map_core/iterators/EllipseIterator.hpp"
#include "grid_map_core/GridMap.hpp"


TEST(EllipseIterator, OneCellWideEllipse)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));

  grid_map::EllipseIterator iterator(map, grid_map::Position(0.0, 0.0), grid_map::Length(8.0, 1.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  ++iterator;
  ++iterator;
  ++iterator;
  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}
