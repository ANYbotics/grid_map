/*
 * GridMapMathTest.cpp
 *
 *  Created on: Feb 10, 2014
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

#include <limits>

#include "grid_map_core/GridMapMath.hpp"


TEST(PositionFromIndex, Simple)
{
  grid_map::Length mapLength(3.0, 2.0);
  grid_map::Position mapPosition(-1.0, 2.0);
  double resolution = 1.0;
  grid_map::Size bufferSize(3, 2);
  grid_map::Position position;

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(0, 0), mapLength, mapPosition, resolution,
      bufferSize));
  EXPECT_DOUBLE_EQ(1.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(1, 0), mapLength, mapPosition, resolution,
      bufferSize));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(1, 1), mapLength, mapPosition, resolution,
      bufferSize));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(2, 1), mapLength, mapPosition, resolution,
      bufferSize));
  EXPECT_DOUBLE_EQ(-1.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.5 + mapPosition.y(), position.y());

  EXPECT_FALSE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(3, 1), mapLength, mapPosition, resolution,
      bufferSize));
}

TEST(PositionFromIndex, CircularBuffer)
{
  grid_map::Length mapLength(0.5, 0.4);
  grid_map::Position mapPosition(-0.1, 13.4);
  double resolution = 0.1;
  grid_map::Size bufferSize(5, 4);
  grid_map::Index bufferStartIndex(3, 1);
  grid_map::Position position;

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(3, 1), mapLength, mapPosition, resolution,
      bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.2 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(4, 2), mapLength, mapPosition, resolution,
      bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.05 + mapPosition.y(), position.y());

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(2, 0), mapLength, mapPosition, resolution,
      bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(-0.2 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(0, 0), mapLength, mapPosition, resolution,
      bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(4, 3), mapLength, mapPosition, resolution,
      bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.05 + mapPosition.y(), position.y());

  EXPECT_FALSE(
    grid_map::getPositionFromIndex(
      position, grid_map::Index(5, 3), mapLength, mapPosition, resolution,
      bufferSize, bufferStartIndex));
}

TEST(IndexFromPosition, Simple)
{
  grid_map::Length mapLength(3.0, 2.0);
  grid_map::Position mapPosition(-12.4, -7.1);
  double resolution = 1.0;
  grid_map::Index bufferSize(3, 2);
  grid_map::Index index;

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(1.0, 0.5) + mapPosition, mapLength, mapPosition,
      resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(-1.0, -0.5) + mapPosition, mapLength,
      mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(0.6, 0.1) + mapPosition, mapLength, mapPosition,
      resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(0.4, -0.1) + mapPosition, mapLength, mapPosition,
      resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(0.4, 0.1) + mapPosition, mapLength, mapPosition,
      resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_FALSE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(4.0, 0.5) + mapPosition, mapLength, mapPosition,
      resolution, bufferSize));
}

TEST(IndexFromPosition, EdgeCases)
{
  grid_map::Length mapLength(3.0, 2.0);
  grid_map::Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  grid_map::Size bufferSize(3, 2);
  grid_map::Index index;

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(0.0, DBL_EPSILON), mapLength, mapPosition,
      resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength,
      mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(-0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength,
      mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_FALSE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(-1.5, 1.0), mapLength, mapPosition, resolution,
      bufferSize));
}

TEST(IndexFromPosition, CircularBuffer)
{
  grid_map::Length mapLength(0.5, 0.4);
  grid_map::Position mapPosition(0.4, -0.9);
  double resolution = 0.1;
  grid_map::Size bufferSize(5, 4);
  grid_map::Index bufferStartIndex(3, 1);
  grid_map::Index index;

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(0.2, 0.15) + mapPosition, mapLength, mapPosition,
      resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(
    grid_map::getIndexFromPosition(
      index, grid_map::Position(0.03, -0.17) + mapPosition, mapLength,
      mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));
}

TEST(checkIfPositionWithinMap, Inside)
{
  grid_map::Length mapLength(50.0, 25.0);
  grid_map::Position mapPosition(11.4, 0.0);

  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(0.0, 0.0) + mapPosition,
      mapLength, mapPosition));
  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(5.0, 5.0) + mapPosition,
      mapLength, mapPosition));
  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(20.0, 10.0) + mapPosition,
      mapLength, mapPosition));
  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(20.0, -10.0) + mapPosition, mapLength,
      mapPosition));
  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(-20.0, 10.0) + mapPosition, mapLength,
      mapPosition));
  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(-20.0, -10.0) + mapPosition, mapLength,
      mapPosition));
}

TEST(checkIfPositionWithinMap, Outside)
{
  grid_map::Length mapLength(10.0, 5.0);
  grid_map::Position mapPosition(-3.0, 145.2);

  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(5.5, 0.0) + mapPosition,
      mapLength, mapPosition));
  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(-5.5, 0.0) + mapPosition,
      mapLength, mapPosition));
  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(-5.5, 3.0) + mapPosition,
      mapLength, mapPosition));
  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(-5.5, -3.0) + mapPosition, mapLength,
      mapPosition));
  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(3.0, 3.0) + mapPosition,
      mapLength, mapPosition));
}

TEST(checkIfPositionWithinMap, EdgeCases)
{
  grid_map::Length mapLength(2.0, 3.0);
  grid_map::Position mapPosition(0.0, 0.0);

  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(1.0, -1.5), mapLength,
      mapPosition));
  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(-1.0, 1.5), mapLength,
      mapPosition));
  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(1.0 + DBL_EPSILON, 1.0),
      mapLength, mapPosition));
  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position((2.0 + DBL_EPSILON) / 2.0, 1.0), mapLength,
      mapPosition));
  EXPECT_FALSE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(0.5, -1.5 - (2.0 * DBL_EPSILON)), mapLength,
      mapPosition));
  EXPECT_TRUE(
    grid_map::checkIfPositionWithinMap(
      grid_map::Position(-0.5, (3.0 + DBL_EPSILON) / 2.0), mapLength,
      mapPosition));
}

TEST(getIndexShiftFromPositionShift, All)
{
  double resolution = 1.0;
  grid_map::Index indexShift;

  EXPECT_TRUE(
    grid_map::getIndexShiftFromPositionShift(
      indexShift, grid_map::Vector(0.0, 0.0),
      resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(
    grid_map::getIndexShiftFromPositionShift(
      indexShift, grid_map::Vector(0.35, -0.45),
      resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(
    grid_map::getIndexShiftFromPositionShift(
      indexShift, grid_map::Vector(0.55, -0.45),
      resolution));
  EXPECT_EQ(-1, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(
    grid_map::getIndexShiftFromPositionShift(
      indexShift, grid_map::Vector(-1.3, -2.65),
      resolution));
  EXPECT_EQ(1, indexShift(0));
  EXPECT_EQ(3, indexShift(1));

  EXPECT_TRUE(
    grid_map::getIndexShiftFromPositionShift(
      indexShift, grid_map::Vector(
        -0.4,
        0.09), 0.2));
  EXPECT_EQ(2, indexShift(0));
  EXPECT_EQ(0, indexShift(1));
}

TEST(getPositionShiftFromIndexShift, All)
{
  double resolution = 0.3;
  grid_map::Vector positionShift;

  EXPECT_TRUE(
    grid_map::getPositionShiftFromIndexShift(
      positionShift, grid_map::Index(0, 0),
      resolution));
  EXPECT_DOUBLE_EQ(0.0, positionShift.x());
  EXPECT_DOUBLE_EQ(0.0, positionShift.y());

  EXPECT_TRUE(
    grid_map::getPositionShiftFromIndexShift(
      positionShift, grid_map::Index(1, -1),
      resolution));
  EXPECT_DOUBLE_EQ(-0.3, positionShift.x());
  EXPECT_DOUBLE_EQ(0.3, positionShift.y());

  EXPECT_TRUE(
    grid_map::getPositionShiftFromIndexShift(
      positionShift, grid_map::Index(2, 1),
      resolution));
  EXPECT_DOUBLE_EQ(-0.6, positionShift.x());
  EXPECT_DOUBLE_EQ(-0.3, positionShift.y());
}

TEST(checkIfIndexInRange, All)
{
  grid_map::Size bufferSize(10, 15);
  EXPECT_TRUE(grid_map::checkIfIndexInRange(grid_map::Index(0, 0), bufferSize));
  EXPECT_TRUE(grid_map::checkIfIndexInRange(grid_map::Index(9, 14), bufferSize));
  EXPECT_FALSE(grid_map::checkIfIndexInRange(grid_map::Index(10, 5), bufferSize));
  EXPECT_FALSE(grid_map::checkIfIndexInRange(grid_map::Index(5, 300), bufferSize));
  EXPECT_FALSE(grid_map::checkIfIndexInRange(grid_map::Index(-1, 0), bufferSize));
  EXPECT_FALSE(grid_map::checkIfIndexInRange(grid_map::Index(0, -300), bufferSize));
}

TEST(boundIndexToRange, All)
{
  int index;
  int bufferSize = 10;

  index = 0;
  grid_map::boundIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 1;
  grid_map::boundIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -1;
  grid_map::boundIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 9;
  grid_map::boundIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 10;
  grid_map::boundIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 35;
  grid_map::boundIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = -19;
  grid_map::boundIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);
}

TEST(wrapIndexToRange, All)
{
  int index;
  int bufferSize = 10;

  index = 0;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 1;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -1;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 9;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 10;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 11;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = 35;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(5, index);

  index = -9;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -19;
  grid_map::wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);
}

TEST(boundPositionToRange, Simple)
{
  double epsilon = 11.0 * std::numeric_limits<double>::epsilon();

  grid_map::Length mapLength(30.0, 10.0);
  grid_map::Position mapPosition(0.0, 0.0);
  grid_map::Position position;

  position << 0.0, 0.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());

  position << 15.0, 5.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 15.0 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 5.0 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -15.0, -5.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 15.0 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 5.0 * epsilon);
  EXPECT_LE(-5.0, position.y());

  position << 16.0, 6.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 16.0 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 6.0 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -16.0, -6.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 16.0 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 6.0 * epsilon);
  EXPECT_LE(-5.0, position.y());

  position << 1e6, 1e6;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 1e6 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 1e6 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -1e6, -1e6;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 1e6 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 1e6 * epsilon);
  EXPECT_LE(-5.0, position.y());
}

TEST(boundPositionToRange, Position)
{
  double epsilon = 11.0 * std::numeric_limits<double>::epsilon();

  grid_map::Length mapLength(30.0, 10.0);
  grid_map::Position mapPosition(1.0, 2.0);
  grid_map::Position position;

  position << 0.0, 0.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());

  position << 16.0, 7.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 16.0 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 7.0 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -14.0, -3.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 14.0 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 3.0 * epsilon);
  EXPECT_LE(-3.0, position.y());

  position << 17.0, 8.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 17.0 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 8.0 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -15.0, -4.0;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 15.0 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 4.0 * epsilon);
  EXPECT_LE(-3.0, position.y());

  position << 1e6, 1e6;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 1e6 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 1e6 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -1e6, -1e6;
  grid_map::boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 1e6 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 1e6 * epsilon);
  EXPECT_LE(-3.0, position.y());
}

TEST(getSubmapInformation, Simple)
{
  // Map
  grid_map::Length mapLength(5.0, 4.0);
  grid_map::Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  grid_map::Size bufferSize(5, 4);

  // Requested submap
  grid_map::Position requestedSubmapPosition;
  grid_map::Position requestedSubmapLength;

  // The returned submap indeces
  grid_map::Index submapTopLeftIndex;
  grid_map::Index submapSize;
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
      requestedSubmapPosition, requestedSubmapLength, mapLength, mapPosition, resolution,
      bufferSize));
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
  grid_map::Length mapLength(5.0, 4.0);
  grid_map::Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  grid_map::Size bufferSize(5, 4);

  // Requested submap
  grid_map::Position requestedSubmapPosition;
  grid_map::Length requestedSubmapLength;

  // The returned submap indeces
  grid_map::Index submapTopLeftIndex;
  grid_map::Index submapSize;
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  requestedSubmapPosition << -1.0, -0.5;
  requestedSubmapLength << 0.0, 0.0;
  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
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
  grid_map::Length mapLength(5.0, 4.0);
  grid_map::Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  grid_map::Size bufferSize(5, 4);

  // Requested submap
  grid_map::Position requestedSubmapPosition;
  grid_map::Length requestedSubmapLength;

  // The returned submap indeces
  grid_map::Index submapTopLeftIndex;
  grid_map::Size submapSize;
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  requestedSubmapPosition << 2.0, 1.5;
  requestedSubmapLength << 2.9, 2.9;
  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
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
  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
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
  grid_map::Length mapLength(5.0, 4.0);
  grid_map::Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  grid_map::Size bufferSize(5, 4);
  grid_map::Index bufferStartIndex(2, 1);

  // Requested submap
  grid_map::Position requestedSubmapPosition;
  grid_map::Length requestedSubmapLength;

  // The returned submap indeces
  grid_map::Index submapTopLeftIndex;
  grid_map::Size submapSize;
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
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
  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
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
  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
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
  grid_map::Length mapLength(4.98, 4.98);
  grid_map::Position mapPosition(-4.98, -5.76);
  double resolution = 0.06;
  grid_map::Size bufferSize(83, 83);
  grid_map::Index bufferStartIndex(0, 13);

  // Requested submap
  grid_map::Position requestedSubmapPosition(-7.44, -3.42);
  grid_map::Length requestedSubmapLength(0.12, 0.12);

  // The returned submap indeces
  grid_map::Index submapTopLeftIndex;
  grid_map::Size submapSize;
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
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
  grid_map::Length mapLength(4.98, 4.98);
  grid_map::Position mapPosition(2.46, -25.26);
  double resolution = 0.06;
  grid_map::Size bufferSize(83, 83);
  grid_map::Index bufferStartIndex(42, 6);

  // Requested submap
  grid_map::Position requestedSubmapPosition(0.24, -26.82);
  grid_map::Length requestedSubmapLength(0.624614, 0.462276);

  // The returned submap indeces
  grid_map::Index submapTopLeftIndex;
  grid_map::Size submapSize;
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  EXPECT_TRUE(
    grid_map::getSubmapInformation(
      submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
      requestedSubmapPosition, requestedSubmapLength,
      mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_LT(0, submapSize(0));
  EXPECT_LT(0, submapSize(1));
  EXPECT_LT(0.0, submapLength(0));
  EXPECT_LT(0.0, submapLength(1));
}

TEST(getBufferRegionsForSubmap, Trivial)
{
  grid_map::Size bufferSize(5, 4);
  grid_map::Index submapIndex(0, 0);
  grid_map::Size submapSize(0, 0);
  std::vector<grid_map::BufferRegion> regions;

  EXPECT_TRUE(grid_map::getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(1u, regions.size());
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(0, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(0, regions[0].getSize()[0]);
  EXPECT_EQ(0, regions[0].getSize()[1]);

  submapSize << 0, 7;
  EXPECT_FALSE(grid_map::getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));

  submapSize << 6, 7;
  EXPECT_FALSE(grid_map::getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
}

TEST(getBufferRegionsForSubmap, Simple)
{
  grid_map::Size bufferSize(5, 4);
  grid_map::Index submapIndex(1, 2);
  grid_map::Size submapSize(3, 2);
  std::vector<grid_map::BufferRegion> regions;

  EXPECT_TRUE(grid_map::getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(1u, regions.size());
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(1, regions[0].getStartIndex()[0]);
  EXPECT_EQ(2, regions[0].getStartIndex()[1]);
  EXPECT_EQ(3, regions[0].getSize()[0]);
  EXPECT_EQ(2, regions[0].getSize()[1]);
}

TEST(getBufferRegionsForSubmap, CircularBuffer)
{
  grid_map::Size bufferSize(5, 4);
  grid_map::Index submapIndex;
  grid_map::Size submapSize;
  grid_map::Index bufferStartIndex(3, 1);
  std::vector<grid_map::BufferRegion> regions;

  submapIndex << 3, 1;
  submapSize << 2, 3;
  EXPECT_TRUE(
    grid_map::getBufferRegionsForSubmap(
      regions, submapIndex, submapSize, bufferSize,
      bufferStartIndex));
  EXPECT_EQ(1u, regions.size());
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(3, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);

  submapIndex << 4, 1;
  submapSize << 2, 3;
  EXPECT_TRUE(
    grid_map::getBufferRegionsForSubmap(
      regions, submapIndex, submapSize, bufferSize,
      bufferStartIndex));
  EXPECT_EQ(2u, regions.size());
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(4, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(1, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::BottomLeft, regions[1].getQuadrant());
  EXPECT_EQ(0, regions[1].getStartIndex()[0]);
  EXPECT_EQ(1, regions[1].getStartIndex()[1]);
  EXPECT_EQ(1, regions[1].getSize()[0]);
  EXPECT_EQ(3, regions[1].getSize()[1]);

  submapIndex << 1, 0;
  submapSize << 2, 1;
  EXPECT_TRUE(
    grid_map::getBufferRegionsForSubmap(
      regions, submapIndex, submapSize, bufferSize,
      bufferStartIndex));
  EXPECT_EQ(1u, regions.size());
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::BottomRight, regions[0].getQuadrant());
  EXPECT_EQ(1, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(1, regions[0].getSize()[1]);

  submapIndex << 3, 1;
  submapSize << 5, 4;
  EXPECT_TRUE(
    grid_map::getBufferRegionsForSubmap(
      regions, submapIndex, submapSize, bufferSize,
      bufferStartIndex)); \
  EXPECT_EQ(4u, regions.size());
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(3, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::TopRight, regions[1].getQuadrant());
  EXPECT_EQ(3, regions[1].getStartIndex()[0]);
  EXPECT_EQ(0, regions[1].getStartIndex()[1]);
  EXPECT_EQ(2, regions[1].getSize()[0]);
  EXPECT_EQ(1, regions[1].getSize()[1]);
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::BottomLeft, regions[2].getQuadrant());
  EXPECT_EQ(0, regions[2].getStartIndex()[0]);
  EXPECT_EQ(1, regions[2].getStartIndex()[1]);
  EXPECT_EQ(3, regions[2].getSize()[0]);
  EXPECT_EQ(3, regions[2].getSize()[1]);
  EXPECT_EQ(grid_map::BufferRegion::Quadrant::BottomRight, regions[3].getQuadrant());
  EXPECT_EQ(0, regions[3].getStartIndex()[0]);
  EXPECT_EQ(0, regions[3].getStartIndex()[1]);
  EXPECT_EQ(3, regions[3].getSize()[0]);
  EXPECT_EQ(1, regions[3].getSize()[1]);
}

TEST(checkIncrementIndex, Simple)
{
  grid_map::Index index(0, 0);
  grid_map::Size bufferSize(4, 3);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(1, index[1]);

  for (int i = 0; i < 6; i++) {
    EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize));
  }
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_FALSE(grid_map::incrementIndex(index, bufferSize));
  EXPECT_EQ(index[0], index[0]);
  EXPECT_EQ(index[1], index[1]);
}

TEST(checkIncrementIndex, CircularBuffer)
{
  grid_map::Size bufferSize(4, 3);
  grid_map::Index bufferStartIndex(2, 1);
  grid_map::Index index(bufferStartIndex);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_FALSE(grid_map::incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(index[0], index[0]);
  EXPECT_EQ(index[1], index[1]);
}

TEST(checkIncrementIndexForSubmap, Simple)
{
  grid_map::Index submapIndex(0, 0);
  grid_map::Index index;
  grid_map::Index submapTopLeftIndex(3, 1);
  grid_map::Size submapBufferSize(2, 4);
  grid_map::Size bufferSize(8, 5);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(1, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(2, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(3, index[1]);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(0, submapIndex[1]);
  EXPECT_EQ(4, index[0]);
  EXPECT_EQ(1, index[1]);

  submapIndex << 1, 2;
  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(4, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_FALSE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize));

  submapIndex << 2, 0;
  EXPECT_FALSE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize));
}

TEST(checkIncrementIndexForSubmap, CircularBuffer)
{
  grid_map::Index submapIndex(0, 0);
  grid_map::Index index;
  grid_map::Index submapTopLeftIndex(6, 3);
  grid_map::Size submapBufferSize(2, 4);
  grid_map::Size bufferSize(8, 5);
  grid_map::Index bufferStartIndex(3, 2);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(1, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(2, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize, bufferStartIndex));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(0, submapIndex[1]);
  EXPECT_EQ(7, index[0]);
  EXPECT_EQ(3, index[1]);

  submapIndex << 1, 2;
  EXPECT_TRUE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize, bufferStartIndex));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(7, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_FALSE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize, bufferStartIndex));

  submapIndex << 2, 0;
  EXPECT_FALSE(
    grid_map::incrementIndexForSubmap(
      submapIndex, index, submapTopLeftIndex, submapBufferSize,
      bufferSize, bufferStartIndex));
}

TEST(getIndexFromLinearIndex, Simple)
{
  EXPECT_TRUE(
    (grid_map::Index(
      0,
      0) == grid_map::getIndexFromLinearIndex(0, grid_map::Size(8, 5), false)).all());
  EXPECT_TRUE(
    (grid_map::Index(
      1,
      0) == grid_map::getIndexFromLinearIndex(1, grid_map::Size(8, 5), false)).all());
  EXPECT_TRUE(
    (grid_map::Index(
      0,
      1) == grid_map::getIndexFromLinearIndex(1, grid_map::Size(8, 5), true)).all());
  EXPECT_TRUE(
    (grid_map::Index(
      2,
      0) == grid_map::getIndexFromLinearIndex(2, grid_map::Size(8, 5), false)).all());
  EXPECT_TRUE(
    (grid_map::Index(
      0,
      1) == grid_map::getIndexFromLinearIndex(8, grid_map::Size(8, 5), false)).all());
  EXPECT_TRUE(
    (grid_map::Index(
      7,
      4) == grid_map::getIndexFromLinearIndex(39, grid_map::Size(8, 5), false)).all());
}
