/*
 * SubmapIteratorTest.cpp
 *
 *  Created on: Sep 15, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

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

TEST(SubmapIterator, Simple) {
  Eigen::Array2i submapTopLeftIndex(3, 1);
  Eigen::Array2i submapBufferSize(3, 2);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Array2d(8.1, 5.1), 1.0, Vector2d(0.0, 0.0));  // bufferSize(8, 5)

  SubmapIterator iterator(map, submapTopLeftIndex, submapBufferSize);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(submapTopLeftIndex(0), (*iterator)(0));
  EXPECT_EQ(submapTopLeftIndex(1), (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));
}

TEST(SubmapIterator, CircularBuffer) {
  Eigen::Array2i submapTopLeftIndex(6, 3);
  Eigen::Array2i submapBufferSize(2, 4);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)
  map.move(Position(-3.0, -2.0));                              // bufferStartIndex(3, 2)

  SubmapIterator iterator(map, submapTopLeftIndex, submapBufferSize);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(submapTopLeftIndex(0), (*iterator)(0));
  EXPECT_EQ(submapTopLeftIndex(1), (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(2, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(2, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));
}

/**
 * The submap should contain the same elements as before even after moving the underlying map.
 *
 *                                                          +----------------------------+
 *                                                          |                            |
 *                                                          |                            |
 *              +----------------------------+              |                            |
 *              |0  0  0  0  0  0  0  0  0  0|              |      0  0  0  0  0  0  0  0|
 *              |     +----+                 |              |           +----+           |
 * Submap       |1  1 |1  1| 1  1  1  1  1  1|              |      1  1 |1  1| 1  1  1  1|
 *           +------> |    |                 |              |           |    |           |
 *              |2  2 |2  2| 2  2  2  2  2  2|              |      2  2 |2  2| 2  2  2  2|
 *              |     +----+                 |              |           +----+           |
 *              |3  3  3  3  3  3  3  3  3  3|   Move       |      3  3  3  3  3  3  3  3|
 *              |                            |              |                            |
 *              |4  4  4  4  4  4  4  4  4  4| +--------->  |      4  4  4  4  4  4  4  4|
 *              |                            |              |                            |
 *              |5  5  5  5  5  5  5  5  5  5|              |      5  5  5  5  5  5  5  5|
 *              |                            |              |                            |
 *              |6  6  6  6  6  6  6  6  6  6|              |      6  6  6  6  6  6  6  6|
 *              |                            |              |                            |
 *              |7  7  7  7  7  7  7  7  7  7|              |      7  7  7  7  7  7  7  7|
 *              |                            |              +----------------------------+
 *              |8  8  8  8  8  8  8  8  8  8|
 *              |                            |
 *              |9  9  9  9  9  9  9  9  9  9|
 *              +----------------------------+
 */
TEST(SubmapIterator, InterleavedExecutionWithMove) {
  grid_map::Index submapTopLeftIndex(3, 1);
  grid_map::Size submapSize(2, 2);

  GridMap map({"layer"});

  map.setGeometry(Length(10, 10), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  auto& layer = map.get("layer");

  // Initialize the layer as sketched.
  for (size_t colIndex = 0; colIndex < layer.cols(); colIndex++) {
    layer.col(colIndex).setConstant(colIndex);
  }

  std::cout << "(4,7) contains " << map.at("layer", {4, 7}) << std::endl;
  // Instantiate the submap iterator as sketched.
  SubmapIterator iterator(map, submapTopLeftIndex, submapSize);

  // check that the submap iterator returns {1,1,2,2}
  auto checkCorrectValues = [](std::array<double, 4> given) {
    int countOnes = 0, countTwos = 0;
    for (auto& value : given) {
      if (std::abs(value - 1.0) < 1e-6) {
        countOnes++;
      } else if (std::abs(value - 2.0) < 1e-6) {
        countTwos++;
      } else {
        FAIL() << "Submap iterator returned unexpected value.";
      }
    }
    EXPECT_EQ(countOnes, 2);
    EXPECT_EQ(countTwos, 2);
  };

  std::array<double, 4> returnedSequence;
  returnedSequence.fill(0);

  for (size_t submapIndex = 0; submapIndex < 4; submapIndex++) {
    returnedSequence.at(submapIndex) = map.at("layer", *iterator);
    ++iterator;
  }

  checkCorrectValues(returnedSequence);

  // Reset the iterator and now check that it still returns the same sequence when we move the map interleaved with iterating.
  iterator = SubmapIterator(map, submapTopLeftIndex, submapSize);
  returnedSequence.fill(0);
  for (size_t submapIndex = 0; submapIndex < 4; submapIndex++) {
    if (submapIndex == 2) {
      // Now move the map as depicted.
      map.move(Position(2.0, 2.0));
    }
    returnedSequence.at(submapIndex) = map.at("layer", *iterator);
    ++iterator;
  }
  checkCorrectValues(returnedSequence);

  // TODO (mwulf, mgaertner): This behavior is not yet implemented:
  //
  //  // Reset the iterator and now check that the iterator throws? if the submap moved out of range.
  //  iterator = SubmapIterator(map, submapTopLeftIndex, submapSize);
  //
  //  EXPECT_ANY_THROW(for (size_t submapIndex = 0; submapIndex < 4; submapIndex++) {
  //    if (submapIndex == 2) {
  //      // Now move the map so that the submap gets out of range.
  //      map.move(Position(20.0, 20.0));
  //    }
  //    returnedSequence.at(submapIndex) = map.at("layer", *iterator);
  //    ++iterator;
  //  });
}