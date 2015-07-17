/*
 * SubmapIteratorTest.cpp
 *
 *  Created on: Sep 15, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/SubmapIterator.hpp"
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

TEST(checkSubmapIterator, Simple)
{
  Eigen::Array2i submapTopLeftIndex(3, 1);
  Eigen::Array2i submapBufferSize(3, 2);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Array2d(8.1, 5.1), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)

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

  // check end pointer

  SubmapIterator it(map, submapTopLeftIndex, submapBufferSize);
  Index last_index;
  for (;!it.isPastEnd();++it) {
    last_index = it.getSubmapIndex();
  }
  Index idxIt = *it;
  SubmapIterator itEnd = it.end();
  Index idxEnd = *itEnd;
  EXPECT_EQ(idxIt(0),idxEnd(0));
  EXPECT_EQ(idxIt(1),idxEnd(1));

  std::cout << idxIt << std::endl;
  std::cout << last_index << std::endl;
  std::cout << idxEnd  << std::endl;
}

TEST(checkSubmapIterator, CircularBuffer)
{
  Eigen::Array2i submapTopLeftIndex(6, 3);
  Eigen::Array2i submapBufferSize(2, 4);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Array2d(8.1, 5.1), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)
  map.move(Vector2d(-3.0, -2.0)); // bufferStartIndex(3, 2)

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

TEST(checkSubmapIterator, OutOfRangeRequest)
{
  Eigen::Array2i submapTopLeftIndex(3, 1);
  Eigen::Array2i submapBufferSize(6, 6);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Array2d(8.1, 5.1), 1.0, Vector2d(0.0, 0.0)); // bufferSize(8, 5)

  SubmapIterator iterator(map, submapTopLeftIndex, submapBufferSize);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(submapTopLeftIndex(0), (*iterator)(0));
  EXPECT_EQ(submapTopLeftIndex(1), (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  EXPECT_EQ(5, iterator.getSubmapSize()(0));
  EXPECT_EQ(4, iterator.getSubmapSize()(1));

  Index lastIndex;
  for (; !iterator.isPastEnd(); ++iterator) {
    lastIndex = *iterator;
  }
  Index idxIt = *iterator;
  SubmapIterator itEnd = iterator.end();
  Index idxEnd = *itEnd;
  EXPECT_EQ(idxIt(0),idxEnd(0));
  EXPECT_EQ(idxIt(1),idxEnd(1));

  std::cout << idxIt << std::endl;
  std::cout << lastIndex << std::endl;
  std::cout << idxEnd  << std::endl;
}
