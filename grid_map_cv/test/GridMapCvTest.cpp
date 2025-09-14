/*
 * GridMapCvTest.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: Peter Fankhauser, Martin Wermelinger
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/gtest_eigen.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// gtest
#include <gtest/gtest.h>

#include <limits>

#include "grid_map_cv/grid_map_cv.hpp"


TEST(ImageConversion, roundTrip8UC3)
{
  // Create grid map.
  grid_map::GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn["layer"].setRandom();
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image.
  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 3>(
    mapIn, "layer", CV_8UC3, minValue,
    maxValue, image);

  // Convert back to grid map.
  grid_map::GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
    image, "layer", mapOut, minValue,
    maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) /
    static_cast<float>(std::numeric_limits<unsigned char>::max());
  grid_map::expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTrip8UC4)
{
  // Create grid map.
  grid_map::GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.1);
  mapIn["layer"].setRandom();
  mapIn["layer"](1, 2) = NAN;  // To check for transparnecy/nan handling.
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image.
  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
    mapIn, "layer", CV_8UC4, minValue,
    maxValue, image);

  // Convert back to grid map.
  grid_map::GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 4>(
    image, "layer", mapOut, minValue,
    maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) /
    static_cast<float>(std::numeric_limits<unsigned char>::max());
  grid_map::expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTrip16UC1)
{
  // Create grid map.
  grid_map::GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn["layer"].setRandom();
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image.
  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<uint16_t, 1>(
    mapIn, "layer", CV_16UC1, minValue, maxValue,
    image);

  // Convert back to grid map.
  grid_map::GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  grid_map::GridMapCvConverter::addLayerFromImage<uint16_t, 1>(
    image, "layer", mapOut, minValue,
    maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) /
    static_cast<float>(std::numeric_limits<unsigned char>::max());
  grid_map::expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTrip32FC1)
{
  // Create grid map.
  grid_map::GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn["layer"].setRandom();
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image.
  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<float, 1>(
    mapIn, "layer", CV_32FC1, minValue, maxValue,
    image);

  // Convert back to grid map.
  grid_map::GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(
    image, "layer", mapOut, minValue,
    maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) /
    static_cast<float>(std::numeric_limits<unsigned char>::max());
  grid_map::expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}
