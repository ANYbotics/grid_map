/*
 * GridMapMathTest.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMapMath.hpp"

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

TEST(PositionFromIndex, Simple)
{
  Array2d mapLength(3.0, 2.0);
  Vector2d mapPosition(-1.0, 2.0);
  double resolution = 1.0;
  Array2i bufferSize(3, 2);
  Vector2d position;

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(0, 0), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(1.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(1, 0), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(1, 1), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(2, 1), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(-1.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.5 + mapPosition.y(), position.y());

  EXPECT_FALSE(getPositionFromIndex(position, Array2i(3, 1), mapLength, mapPosition, resolution, bufferSize));
}

TEST(PositionFromIndex, CircularBuffer)
{
  Array2d mapLength(0.5, 0.4);
  Vector2d mapPosition(-0.1, 13.4);
  double resolution = 0.1;
  Array2i bufferSize(5, 4);
  Array2i bufferStartIndex(3, 1);
  Vector2d position;

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(3, 1), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.2 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(4, 2), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.05 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(2, 0), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(-0.2 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(0, 0), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(4, 3), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.05 + mapPosition.y(), position.y());

  EXPECT_FALSE(getPositionFromIndex(position, Array2i(5, 3), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
}

TEST(IndexFromPosition, Simple)
{
  Array2d mapLength(3.0, 2.0);
  Vector2d mapPosition(-12.4, -7.1);
  double resolution = 1.0;
  Array2i bufferSize(3, 2);
  Array2i index;

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(1.0, 0.5) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(-1.0, -0.5) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.6, 0.1) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.4, -0.1) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.4, 0.1) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_FALSE(getIndexFromPosition(index, Vector2d(4.0, 0.5) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
}

TEST(IndexFromPosition, EdgeCases)
{
  Array2d mapLength(3.0, 2.0);
  Vector2d mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Array2i bufferSize(3, 2);
  Array2i index;

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.0, DBL_EPSILON), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(-0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));
}

TEST(IndexFromPosition, CircularBuffer)
{
  Array2d mapLength(0.5, 0.4);
  Vector2d mapPosition(0.4, -0.9);
  double resolution = 0.1;
  Array2i bufferSize(5, 4);
  Array2i bufferStartIndex(3, 1);
  Array2i index;

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.2, 0.15) + mapPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.03, -0.17) + mapPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));
}

TEST(checkIfPositionWithinMap, Inside)
{
  Array2d mapLength(50.0, 25.0);
  Vector2d mapPosition(11.4, 0.0);

  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(0.0, 0.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(5.0, 5.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(20.0, 10.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(20.0, -10.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-20.0, 10.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-20.0, -10.0) + mapPosition, mapLength, mapPosition));
}

TEST(checkIfPositionWithinMap, Outside)
{
  Array2d mapLength(10.0, 5.0);
  Vector2d mapPosition(-3.0, 145.2);

  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(5.5, 0.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(-5.5, 0.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(-5.5, 3.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(-5.5, -3.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(3.0, 3.0) + mapPosition, mapLength, mapPosition));
}

TEST(checkIfPositionWithinMap, EdgeCases)
{
  Array2d mapLength(2.0, 3.0);
  Vector2d mapPosition(0.0, 0.0);

  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(1.0, -1.5), mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-1.0, 1.5), mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(1.0 + DBL_EPSILON, 1.0), mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d((2.0 + DBL_EPSILON) / 2.0, 1.0), mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(0.5, -1.5 - (2.0 * DBL_EPSILON)), mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-0.5, (3.0 + DBL_EPSILON) / 2.0), mapLength, mapPosition));
}

TEST(getIndexShiftFromPositionShift, All)
{
  double resolution = 1.0;
  Array2i indexShift;

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(0.0, 0.0), resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(0.35, -0.45), resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(0.55, -0.45), resolution));
  EXPECT_EQ(-1, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(-1.3, -2.65), resolution));
  EXPECT_EQ(1, indexShift(0));
  EXPECT_EQ(3, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(-0.4, 0.09), 0.2));
  EXPECT_EQ(2, indexShift(0));
  EXPECT_EQ(0, indexShift(1));
}

TEST(getPositionShiftFromIndexShift, All)
{
  double resolution = 0.3;
  Vector2d positionShift;

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Array2i(0, 0), resolution));
  EXPECT_DOUBLE_EQ(0.0, positionShift.x());
  EXPECT_DOUBLE_EQ(0.0, positionShift.y());

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Array2i(1, -1), resolution));
  EXPECT_DOUBLE_EQ(-0.3, positionShift.x());
  EXPECT_DOUBLE_EQ(0.3, positionShift.y());

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Array2i(2, 1), resolution));
  EXPECT_DOUBLE_EQ(-0.6, positionShift.x());
  EXPECT_DOUBLE_EQ(-0.3, positionShift.y());
}

TEST(checkIfIndexWithinRange, All)
{
  Array2i bufferSize(10, 15);
  EXPECT_TRUE(checkIfIndexWithinRange(Array2i(0, 0), bufferSize));
  EXPECT_TRUE(checkIfIndexWithinRange(Array2i(9, 14), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(10, 5), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(5, 300), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(-1, 0), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(0, -300), bufferSize));
}

TEST(mapIndexWithinRange, All)
{
  int index;
  int bufferSize = 10;

  index = 0;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 1;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -1;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 9;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 10;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 11;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = 35;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(5, index);

  index = -9;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -19;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);
}

TEST(limitPositionToRange, Simple)
{
  double epsilon = 11.0 * numeric_limits<double>::epsilon();

  Array2d mapLength(30.0, 10.0);
  Vector2d mapPosition(0.0, 0.0);
  Vector2d position;

  position << 0.0, 0.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());

  position << 15.0, 5.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 15.0 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 5.0 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -15.0, -5.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 15.0 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 5.0 * epsilon);
  EXPECT_LE(-5.0, position.y());

  position << 16.0, 6.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 16.0 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 6.0 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -16.0, -6.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 16.0 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 6.0 * epsilon);
  EXPECT_LE(-5.0, position.y());

  position << 1e6, 1e6;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 1e6 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 1e6 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -1e6, -1e6;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 1e6 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 1e6 * epsilon);
  EXPECT_LE(-5.0, position.y());
}

TEST(limitPositionToRange, Position)
{
  double epsilon = 11.0 * numeric_limits<double>::epsilon();

  Array2d mapLength(30.0, 10.0);
  Vector2d mapPosition(1.0, 2.0);
  Vector2d position;

  position << 0.0, 0.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());

  position << 16.0, 7.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 16.0 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 7.0 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -14.0, -3.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 14.0 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 3.0 * epsilon);
  EXPECT_LE(-3.0, position.y());

  position << 17.0, 8.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 17.0 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 8.0 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -15.0, -4.0;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 15.0 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 4.0 * epsilon);
  EXPECT_LE(-3.0, position.y());

  position << 1e6, 1e6;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 1e6 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 1e6 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -1e6, -1e6;
  limitPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 1e6 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 1e6 * epsilon);
  EXPECT_LE(-3.0, position.y());
}

TEST(getSubmapInformation, Simple)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  Vector2d mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                           requestedSubmapPosition, requestedSubmapLength, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.5, submapPosition.y());
  EXPECT_DOUBLE_EQ(1.0, submapLength(0));
  EXPECT_DOUBLE_EQ(3.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(1, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, Zero)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  Vector2d mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << -1.0, -0.5;
  requestedSubmapLength << 0.0, 0.0;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(3, submapTopLeftIndex(0));
  EXPECT_EQ(2, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(1, submapSize(1));
  EXPECT_DOUBLE_EQ(requestedSubmapPosition.x(), submapPosition.x());
  EXPECT_DOUBLE_EQ(requestedSubmapPosition.y(), submapPosition.y());
  EXPECT_DOUBLE_EQ(resolution, submapLength(0));
  EXPECT_DOUBLE_EQ(resolution, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, ExceedingBoundaries)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  Vector2d mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << 2.0, 1.5;
  requestedSubmapLength << 2.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(2, submapSize(1));
  EXPECT_DOUBLE_EQ(1.5, submapPosition.x());
  EXPECT_DOUBLE_EQ(1.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(2.0, submapLength(0));
  EXPECT_DOUBLE_EQ(2.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));

  requestedSubmapPosition << 0.0, 0.0;
  requestedSubmapLength << 1e6, 1e6;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(bufferSize(0), submapSize(0));
  EXPECT_EQ(bufferSize(1), submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(mapLength(0), submapLength(0));
  EXPECT_DOUBLE_EQ(mapLength(1), submapLength(1));
  EXPECT_EQ(2, requestedIndexInSubmap(0));
  EXPECT_LE(1, requestedIndexInSubmap(1));
  EXPECT_GE(2, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, CircularBuffer)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  Vector2d mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);
  Array2i bufferStartIndex(2, 1);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(4, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.5, submapPosition.y());
  EXPECT_DOUBLE_EQ(1.0, submapLength(0));
  EXPECT_DOUBLE_EQ(3.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(1, requestedIndexInSubmap(1));

  requestedSubmapPosition << 2.0, 1.5;
  requestedSubmapLength << 2.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(2, submapSize(1));
  EXPECT_DOUBLE_EQ(1.5, submapPosition.x());
  EXPECT_DOUBLE_EQ(1.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(2.0, submapLength(0));
  EXPECT_DOUBLE_EQ(2.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));

  requestedSubmapPosition << 0.0, 0.0;
  requestedSubmapLength << 1e6, 1e6;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(bufferSize(0), submapSize(0));
  EXPECT_EQ(bufferSize(1), submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(mapLength(0), submapLength(0));
  EXPECT_DOUBLE_EQ(mapLength(1), submapLength(1));
  EXPECT_EQ(2, requestedIndexInSubmap(0));
  EXPECT_LE(1, requestedIndexInSubmap(1));
  EXPECT_GE(2, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, Debug1)
{
  // Map
  Array2d mapLength(4.98, 4.98);
  Vector2d mapPosition(-4.98, -5.76);
  double resolution = 0.06;
  Array2i bufferSize(83, 83);
  Array2i bufferStartIndex(0, 13);

  // Requested submap
  Vector2d requestedSubmapPosition(-7.44, -3.42);
  Vector2d requestedSubmapLength(0.12, 0.12);

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.12, submapLength(0));
  EXPECT_DOUBLE_EQ(0.18, submapLength(1));
}

TEST(getSubmapInformation, Debug2)
{
  // Map
  Array2d mapLength(4.98, 4.98);
  Vector2d mapPosition(2.46, -25.26);
  double resolution = 0.06;
  Array2i bufferSize(83, 83);
  Array2i bufferStartIndex(42, 6);

  // Requested submap
  Vector2d requestedSubmapPosition(0.24, -26.82);
  Vector2d requestedSubmapLength(0.624614, 0.462276);

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_LT(0, submapSize(0));
  EXPECT_LT(0, submapSize(1));
  EXPECT_LT(0.0, submapLength(0));
  EXPECT_LT(0.0, submapLength(1));
}

TEST(getBufferRegionsForSubmap, Trivial)
{
  Size bufferSize(5, 4);
  Index submapIndex(0, 0);
  Size submapSize(0, 0);
  std::vector<BufferRegion> regions;

  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(0, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(0, regions[0].getSize()[0]);
  EXPECT_EQ(0, regions[0].getSize()[1]);

  submapSize << 0, 7;
  EXPECT_FALSE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));

  submapSize << 6, 7;
  EXPECT_FALSE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
}

TEST(getBufferRegionsForSubmap, Simple)
{
  Size bufferSize(5, 4);
  Index submapIndex(1, 2);
  Size submapSize(3, 2);
  std::vector<BufferRegion> regions;

  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(1, regions[0].getStartIndex()[0]);
  EXPECT_EQ(2, regions[0].getStartIndex()[1]);
  EXPECT_EQ(3, regions[0].getSize()[0]);
  EXPECT_EQ(2, regions[0].getSize()[1]);
}

TEST(getBufferRegionsForSubmap, CircularBuffer)
{
  Size bufferSize(5, 4);
  Index submapIndex;
  Size submapSize;
  Index bufferStartIndex(3, 1);
  std::vector<BufferRegion> regions;

  submapIndex << 3, 1;
  submapSize << 2, 3;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(3, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);

  submapIndex << 4, 1;
  submapSize << 2, 3;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(4, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(1, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::BottomLeft, regions[1].getQuadrant());
  EXPECT_EQ(0, regions[1].getStartIndex()[0]);
  EXPECT_EQ(1, regions[1].getStartIndex()[1]);
  EXPECT_EQ(1, regions[1].getSize()[0]);
  EXPECT_EQ(3, regions[1].getSize()[1]);

  submapIndex << 1, 0;
  submapSize << 2, 1;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::BottomRight, regions[0].getQuadrant());
  EXPECT_EQ(1, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(1, regions[0].getSize()[1]);

  submapIndex << 3, 1;
  submapSize << 5, 4;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));\
  EXPECT_EQ(4, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(3, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::TopRight, regions[1].getQuadrant());
  EXPECT_EQ(3, regions[1].getStartIndex()[0]);
  EXPECT_EQ(0, regions[1].getStartIndex()[1]);
  EXPECT_EQ(2, regions[1].getSize()[0]);
  EXPECT_EQ(1, regions[1].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::BottomLeft, regions[2].getQuadrant());
  EXPECT_EQ(0, regions[2].getStartIndex()[0]);
  EXPECT_EQ(1, regions[2].getStartIndex()[1]);
  EXPECT_EQ(3, regions[2].getSize()[0]);
  EXPECT_EQ(3, regions[2].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::BottomRight, regions[3].getQuadrant());
  EXPECT_EQ(0, regions[3].getStartIndex()[0]);
  EXPECT_EQ(0, regions[3].getStartIndex()[1]);
  EXPECT_EQ(3, regions[3].getSize()[0]);
  EXPECT_EQ(1, regions[3].getSize()[1]);
}

TEST(checkIncrementIndex, Simple)
{
  Eigen::Array2i index(0, 0);
  Eigen::Array2i bufferSize(4, 3);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(1, index[1]);

  for (int i = 0; i < 6; i++) {
    EXPECT_TRUE(incrementIndex(index, bufferSize));
  }
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_FALSE(incrementIndex(index, bufferSize));
  EXPECT_EQ(index[0], index[0]);
  EXPECT_EQ(index[1], index[1]);
}

TEST(checkIncrementIndex, CircularBuffer)
{
  Eigen::Array2i bufferSize(4, 3);
  Eigen::Array2i bufferStartIndex(2, 1);
  Eigen::Array2i index(bufferStartIndex);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_FALSE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(index[0], index[0]);
  EXPECT_EQ(index[1], index[1]);
}

TEST(checkIncrementIndexForSubmap, Simple)
{
  Eigen::Array2i submapIndex(0, 0);
  Eigen::Array2i index;
  Eigen::Array2i submapTopLeftIndex(3, 1);
  Eigen::Array2i submapBufferSize(2, 4);
  Eigen::Array2i bufferSize(8, 5);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(1, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(2, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(3, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(0, submapIndex[1]);
  EXPECT_EQ(4, index[0]);
  EXPECT_EQ(1, index[1]);

  submapIndex << 1, 2;
  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(4, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));

  submapIndex << 2, 0;
  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
}

TEST(checkIncrementIndexForSubmap, CircularBuffer)
{
  Eigen::Array2i submapIndex(0, 0);
  Eigen::Array2i index;
  Eigen::Array2i submapTopLeftIndex(6, 3);
  Eigen::Array2i submapBufferSize(2, 4);
  Eigen::Array2i bufferSize(8, 5);
  Eigen::Array2i bufferStartIndex(3, 2);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(1, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(2, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(0, submapIndex[1]);
  EXPECT_EQ(7, index[0]);
  EXPECT_EQ(3, index[1]);

  submapIndex << 1, 2;
  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(7, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));

  submapIndex << 2, 0;
  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
}

TEST(getIndexFromLinearIndex, Simple)
{
  EXPECT_TRUE((Index(0, 0) == getIndexFromLinearIndex(0, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(1, 0) == getIndexFromLinearIndex(1, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(0, 1) == getIndexFromLinearIndex(1, Size(8, 5), true)).all());
  EXPECT_TRUE((Index(2, 0) == getIndexFromLinearIndex(2, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(0, 1) == getIndexFromLinearIndex(8, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(7, 4) == getIndexFromLinearIndex(39, Size(8, 5), false)).all());
}
