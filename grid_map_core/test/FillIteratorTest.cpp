/*
 * FillIteratorTest.cpp
 *
 *  Created on: Oct 2, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/iterators/FillIterator.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/IndexCheckerZero.hpp"
#include "grid_map_core/IndexCheckerNonZero.hpp"

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


static bool indexSetContains(const std::vector<grid_map::Index>& index_set, const grid_map::Index& grid_index){

  for (const grid_map::Index& current_index: index_set){
    if (grid_index.isApprox(current_index)){
      return true;
    }
  }

  return false;
}

TEST(FillIterator, FillEntireMap)
{
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  std::shared_ptr<IndexCheckerZero> checker = make_shared<IndexCheckerZero>(map, "data");

  std::vector<grid_map::Index> traveled_set;

  FillIterator iterator(map, Position(0.0, 0.0), checker);

  size_t iteration_count = 0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    ASSERT_FALSE(iteration_count > 4) << "FillIterator is trying to iterate through more spaces than on the GridMap!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "FillIterator revisited a space!";

    traveled_set.push_back(*iterator);
  }

  EXPECT_EQ( iteration_count, 4 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );
}

TEST(FillIterator, HalfSpace)
{
  // Creates a 3x3 grid with a diagonal line splitting the grid.
  GridMap map;
  map.setGeometry(Length(3.0, 3.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);
  map.atPosition("data", grid_map::Position(-1.0,-1.0)) = 2.0;
  map.atPosition("data", grid_map::Position(0.0,0.0)) = 2.0;
  map.atPosition("data", grid_map::Position(1.0,1.0)) = 2.0;

  std::shared_ptr<IndexCheckerZero> checker = make_shared<IndexCheckerZero>(map, "data");

  std::vector<grid_map::Index> traveled_set;

  FillIterator iterator(map, Position(-1.0, 1.0), checker);

  size_t iteration_count = 0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 3) << "FillIterator is trying to iterate through more spaces than in the connected region (assuming a 4-connected fill)!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "FillIterator revisited a space!";

    traveled_set.push_back(*iterator);

  }

  EXPECT_EQ( iteration_count, 3 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(2,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(2,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );

}

TEST(FillIterator, OppositeHalfSpace)
{
  // Creates a 3x3 grid with a diagonal line splitting the grid.
  GridMap map;
  map.setGeometry(Length(3.0, 3.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);
  map.atPosition("data", grid_map::Position(-1.0,-1.0)) = 2.0;
  map.atPosition("data", grid_map::Position(0.0,0.0)) = 2.0;
  map.atPosition("data", grid_map::Position(1.0,1.0)) = 2.0;

  std::shared_ptr<IndexCheckerZero> checker = make_shared<IndexCheckerZero>(map, "data");

  std::vector<grid_map::Index> traveled_set;

  FillIterator iterator(map, Position(1.0, -1.0), checker);

  for ( size_t iteration_count = 0; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 3) << "FillIterator is trying to iterate through more spaces than in the connected region (assuming a 4-connected fill)!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "FillIterator revisited a space!";

    traveled_set.push_back(*iterator);

  }

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,2) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,2) ) );

}




