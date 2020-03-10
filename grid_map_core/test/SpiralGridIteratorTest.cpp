/*
 * SpiralGridIteratorTest.cpp
 *
 *  Created on: Oct 24, 2017
 *      Author: Perry Franklin
 */



#include "grid_map_core/iterators/SpiralGridIterator.hpp"
#include "grid_map_core/GridMap.hpp"

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
using namespace grid_map;

// Helper function to check if a vector of Indices contains a particular index.
static bool indexSetContains(const std::vector<grid_map::Index>& index_set, const grid_map::Index& grid_index){

  for (const grid_map::Index& current_index: index_set){
    if (grid_index.isApprox(current_index)){
      return true;
    }
  }

  return false;
}

// Uses the SpiralGridIterator to fill the entire map.
TEST(SpiralGridIterator, FillEntireMap)
{
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  std::vector<grid_map::Index> traveled_set;

  SpiralGridIterator iterator(map, grid_map::Index(0, 0));

  size_t iteration_count = 0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 4) << "SpiralGridIterator is trying to iterate through more spaces than on the GridMap!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "SpiralGridIterator revisited a space!";

    traveled_set.push_back(*iterator);
  }

  EXPECT_EQ( iteration_count, 4 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );
}

// Uses the SpiralGridIterator to fill the entire map, starting from the opposite corner.
TEST(SpiralGridIterator, FillEntireMapOppositeCorner)
{
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  std::vector<grid_map::Index> traveled_set;

  SpiralGridIterator iterator(map, grid_map::Index(1, 1));

  size_t iteration_count = 0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 4) << "SpiralGridIterator is trying to iterate through more spaces than on the GridMap!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "SpiralGridIterator revisited a space!";

    traveled_set.push_back(*iterator);
  }

  EXPECT_EQ( iteration_count, 4 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );
}

// Uses the SpiralGridIterator to fill a portion of the map (specifically a 3x3 block of the 4x4 map).
TEST(SpiralGridIterator, PartialMapOverlap)
{
  // Creates a 3x3 grid with a diagonal line splitting the grid.
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  std::vector<grid_map::Index> traveled_set;

  SpiralGridIterator iterator(map, grid_map::Index(0, 0), 2);

  size_t iteration_count = 0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 9) << "SpiralGridIterator is trying to iterate through more spaces than it should!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "SpiralGridIterator revisited a space!";

    traveled_set.push_back(*iterator);

  }

  EXPECT_EQ( iteration_count, 9 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,2) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,2) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(2,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(2,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(2,2) ) );

}

// Checks that the SpiralGridIterator does the correct iterations when starting outside the map.
TEST(SpiralGridIterator, CenterOutsideMap)
{
  // Creates a 3x3 grid with a diagonal line splitting the grid.
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  std::vector<grid_map::Index> traveled_set;

  SpiralGridIterator iterator(map, grid_map::Index(2, 2), 8);

  size_t iteration_count = 0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 4) << "SpiralGridIterator is trying to iterate through more spaces than it should!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "SpiralGridIterator revisited a space!";

    traveled_set.push_back(*iterator);

  }

  EXPECT_EQ( iteration_count, 4 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );

}

