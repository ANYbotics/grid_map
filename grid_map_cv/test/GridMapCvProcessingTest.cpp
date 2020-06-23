/*
 * GridMapCvProcessingTest.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Peter Fankhauser
 */

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/gtest_eigen.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// gtest
#include <gtest/gtest.h>

#include "grid_map_cv/grid_map_cv.hpp"

TEST(GridMapCvProcessing, changeResolution)
{
  // Create grid map.
  grid_map::GridMap mapIn;
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn.add("layer", 1.0);
  for (grid_map::CircleIterator iterator(mapIn, mapIn.getPosition(), 0.2); !iterator.isPastEnd();
    ++iterator)
  {
    mapIn.at("layer", *iterator) = 2.0;
  }

  // Change resolution.
  grid_map::GridMap mapOut;
  EXPECT_TRUE(grid_map::GridMapCvProcessing::changeResolution(mapIn, mapOut, 0.1));

  // Check data.
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE(mapIn.getPosition() == mapOut.getPosition());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize() * 10).all());
  EXPECT_EQ(mapIn["layer"](0, 0), mapOut["layer"](0, 0));  // Corner.
  EXPECT_EQ(
    mapIn.atPosition("layer", mapIn.getPosition()),
    mapOut.atPosition("layer", mapOut.getPosition()));  // Center.
}

TEST(GridMapCvProcessing, changeResolutionForMovedMap)
{
  // Create grid map.
  grid_map::GridMap mapIn;
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  grid_map::Position position(0.3, 0.4);
  mapIn.move(position);
  mapIn.add("layer", 1.0);
  for (grid_map::CircleIterator iterator(mapIn, position, 0.2); !iterator.isPastEnd(); ++iterator) {
    mapIn.at("layer", *iterator) = 2.0;
  }

  // Change resolution.
  grid_map::GridMap mapOut;
  EXPECT_TRUE(grid_map::GridMapCvProcessing::changeResolution(mapIn, mapOut, 0.1));

  // Check data.
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE(mapIn.getPosition() == mapOut.getPosition());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize() * 10).all());
  EXPECT_EQ(mapIn["layer"](0, 0), mapOut["layer"](0, 0));  // Corner.
  EXPECT_EQ(
    mapIn.atPosition("layer", mapIn.getPosition()),
    mapOut.atPosition("layer", mapOut.getPosition()));  // Center.
}
