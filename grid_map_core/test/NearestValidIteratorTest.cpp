/*
 * NearestValidIteratorTest.cpp
 *
 *  Created on: Oct 26, 2017
 *      Author: Perry Franklin
 */


#include "grid_map_core/iterators/NearestValidIterator.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/IndexCheckerZero.hpp"

// Eigen
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

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

// Fills the entire map, starting in the first quadrant (position is positive x, positive y)
TEST(NearestValidIterator, FirstQuadrant)
{
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  IndexCheckerZero checker(map, "data");

  std::vector<grid_map::Index> traveled_set;

  Position starting_position(1.0,1.0);

  NearestValidIterator iterator(map, starting_position, checker);

  size_t iteration_count = 0;

  double last_distance = 0.0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 4) << "NearestValidIterator is trying to iterate through more spaces than on the GridMap!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "NearestValidIterator revisited a space!";

    Position index_position;
    ASSERT_TRUE(map.getPosition(*iterator, index_position)) << "For some reason getPosition failed; did the iterator try to check an index outside the grid?" ;
    double distance_to_index = (index_position - starting_position).norm();
    EXPECT_TRUE(distance_to_index >= last_distance) << "The distance to the current index is less than the last index; so we didn't check the nearest first.";

    last_distance = distance_to_index;

    traveled_set.push_back(*iterator);
  }

  EXPECT_EQ( iteration_count, 4 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );
}

// Fills the entire map, starting in the third quadrant (position is negative x, negative y)

TEST(NearestValidIteratorTest, ThirdQuadrant)
{
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  IndexCheckerZero checker(map, "data");

  std::vector<grid_map::Index> traveled_set;

  Position starting_position(-1.0,-1.0);

  NearestValidIterator iterator(map, starting_position, checker);

  size_t iteration_count = 0;

  double last_distance = 0.0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 4) << "NearestValidIterator is trying to iterate through more spaces than on the GridMap!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "NearestValidIterator revisited a space!";

    Position index_position;
    ASSERT_TRUE(map.getPosition(*iterator, index_position)) << "For some reason getPosition failed; did the iterator try to check an index outside the grid?" ;
    double distance_to_index = (index_position - starting_position).norm();
    EXPECT_TRUE(distance_to_index >= last_distance) << "The distance to the current index is less than the last index; so we didn't check the nearest first.";

    last_distance = distance_to_index;

    traveled_set.push_back(*iterator);
  }

  EXPECT_EQ( iteration_count, 4 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );
}

TEST(NearestValidIterator, OffCenter)
{
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 0.0);

  IndexCheckerZero checker(map, "data");

  std::vector<grid_map::Index> traveled_set;

  Position starting_position(0.1,0.1);

  NearestValidIterator iterator(map, starting_position, checker);

  size_t iteration_count = 0;

  double last_distance = 0.0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 4) << "NearestValidIterator is trying to iterate through more spaces than on the GridMap!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "NearestValidIterator revisited a space!";

    Position index_position;
    ASSERT_TRUE(map.getPosition(*iterator, index_position)) << "For some reason getPosition failed; did the iterator try to check an index outside the grid?" ;
    double distance_to_index = (index_position - starting_position).norm();
    EXPECT_TRUE(distance_to_index >= last_distance) << "The distance to the current index is less than the last index; so we didn't check the nearest first.";

    last_distance = distance_to_index;

    traveled_set.push_back(*iterator);
  }

  EXPECT_EQ( iteration_count, 4 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,1) ) );
}

// Checks that NearestValidIterator only iterates over valid cells.
TEST(NearestValidIterator, OnlyValid)
{
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("data", 1.0);

  map.at("data", grid_map::Index(0,1)) = 0.0;
  map.at("data", grid_map::Index(1,0)) = 0.0;

  IndexCheckerZero checker(map, "data");

  std::vector<grid_map::Index> traveled_set;

  Position starting_position(-1.0,-1.0);

  NearestValidIterator iterator(map, starting_position, checker);

  size_t iteration_count = 0;

  double last_distance = 0.0;
  for ( ; !iterator.isPastEnd(); ++iterator){
    iteration_count++;
    EXPECT_FALSE(iteration_count > 2) << "NearestValidIterator is trying to iterate through more spaces than Valid!";

    EXPECT_FALSE( indexSetContains(traveled_set, *iterator) ) << "NearestValidIterator revisited a space!";

    Position index_position;
    ASSERT_TRUE(map.getPosition(*iterator, index_position)) << "For some reason getPosition failed; did the iterator try to check an index outside the grid?" ;
    double distance_to_index = (index_position - starting_position).norm();
    EXPECT_TRUE(distance_to_index >= last_distance) << "The distance to the current index is less than the last index; so we didn't check the nearest first.";

    last_distance = distance_to_index;

    traveled_set.push_back(*iterator);
  }

  EXPECT_EQ( iteration_count, 2 );

  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(0,1) ) );
  EXPECT_TRUE( indexSetContains(traveled_set, grid_map::Index(1,0) ) );
}
