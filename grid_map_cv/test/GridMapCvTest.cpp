/*
 * GridMapCvTest.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: Peter Fankhauser, Martin Wermelinger
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_cv/grid_map_cv.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/gtest_eigen.hpp>

// gtest
#include <gtest/gtest.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace grid_map;

void replaceNan(Matrix& m, const double newValue)
{
  for(int r = 0; r < m.rows(); r++)
  {
    for(int c = 0; c < m.cols(); c++)
    {
      if (std::isnan(m(r,c)))
      {
        m(r,c) = newValue;
      }
    }
  }
}

TEST(ImageConversion, roundTrip8UC3)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.1);
  mapIn["layer"].setRandom(); // Sets the layer to random values in [-1.0, 1.0].
  mapIn.move(Position(0.5, -0.2));
  const float minValue = -1.0;
  const float maxValue = 1.0;
  replaceNan(mapIn.get("layer"), minValue); // When we move `mapIn`, new areas are filled with NaN. As `toImage` does not support NaN, we replace NaN with `minValue` instead.

  // Convert to image.
  cv::Mat image;
  GridMapCvConverter::toImage<unsigned char, 3>(mapIn, "layer", CV_8UC3, minValue, maxValue, image);

  // Convert back to grid map.
  GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  GridMapCvConverter::addLayerFromImage<unsigned char, 3>(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) / (float) std::numeric_limits<unsigned char>::max();
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTrip8UC4)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.1);
  mapIn["layer"].setRandom(); // Sets the layer to random values in [-1.0, 1.0].
  mapIn.move(Position(0.5, -0.2));
  const float minValue = -1.0;
  const float maxValue = 1.0;
  replaceNan(mapIn.get("layer"), minValue); // When we move `mapIn`, new areas are filled with NaN. As `toImage` does not support NaN, we replace NaN with `minValue` instead.

  // Convert to image.
  cv::Mat image;
  GridMapCvConverter::toImage<unsigned char, 4>(mapIn, "layer", CV_8UC4, minValue, maxValue, image);

  // Convert back to grid map.
  GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  GridMapCvConverter::addLayerFromImage<unsigned char, 4>(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) / (float) std::numeric_limits<unsigned char>::max();
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTrip16UC1)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.1);
  mapIn["layer"].setRandom(); // Sets the layer to random values in [-1.0, 1.0].
  mapIn.move(Position(0.5, -0.2));
  const float minValue = -1.0;
  const float maxValue = 1.0;
  replaceNan(mapIn.get("layer"), minValue); // When we move `mapIn`, new areas are filled with NaN. As `toImage` does not support NaN, we replace NaN with `minValue` instead.

  // Convert to image.
  cv::Mat image;
  GridMapCvConverter::toImage<unsigned short, 1>(mapIn, "layer", CV_16UC1, minValue, maxValue, image);

  // Convert back to grid map.
  GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) / (float) std::numeric_limits<unsigned char>::max();
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTrip32FC1)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.1);
  mapIn["layer"].setRandom(); // Sets the layer to random values in [-1.0, 1.0].
  mapIn.move(Position(0.5, -0.2));
  const float minValue = -1.0;
  const float maxValue = 1.0;
  replaceNan(mapIn.get("layer"), minValue); // When we move `mapIn`, new areas are filled with NaN. As `toImage` does not support NaN, we replace NaN with `minValue` instead.

  // Convert to image.
  cv::Mat image;
  GridMapCvConverter::toImage<float, 1>(mapIn, "layer", CV_32FC1, minValue, maxValue, image);

  // Convert back to grid map.
  GridMap mapOut(mapIn);
  mapOut["layer"].setConstant(NAN);
  GridMapCvConverter::addLayerFromImage<float, 1>(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) / (float) std::numeric_limits<unsigned char>::max();
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}
