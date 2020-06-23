/*
 * SpiralIteratorTest.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Benjamin Scholz
 *	 Institute: University of Hamburg, TAMS
 */

// Eigen
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>

#include "grid_map_core/iterators/SpiralIterator.hpp"
#include "grid_map_core/GridMap.hpp"


TEST(SpiralIterator, CenterOutOfMap)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(0.0, 0.0));
  grid_map::Position center(8.0, 0.0);
  double radius = 5.0;

  grid_map::SpiralIterator iterator(map, center, radius);

  grid_map::Position iterator_position;
  map.getPosition(*iterator, iterator_position);

  EXPECT_TRUE(map.isInside(iterator_position));
}
