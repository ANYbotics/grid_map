/*
 * CostmapConverterTest.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/CostmapConverter.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

using namespace grid_map;

TEST(Costmap2DConversion, initializeFrom)
{
  CostmapConverter<GridMap> costmapConverter;
  // Create Costmap2D.
  costmap_2d::Costmap2D costmap2d(8, 5, 1.0, 2.0, 3.0);

  // Convert to grid map.
  GridMap gridMap;
  costmapConverter.initializeFromCostmap2d(costmap2d, gridMap);

  // Check map info.
  // Different conventions: Costmap2d returns the *centerpoint* of the last cell in the map.
  Length length = gridMap.getLength() - Length::Constant(0.5 * gridMap.getResolution());
  Length position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  EXPECT_EQ(costmap2d.getSizeInMetersX(), length.x());
  EXPECT_EQ(costmap2d.getSizeInMetersY(), length.y());
  EXPECT_EQ(costmap2d.getSizeInCellsX(), gridMap.getSize()[0]);
  EXPECT_EQ(costmap2d.getSizeInCellsY(), gridMap.getSize()[1]);
  EXPECT_EQ(costmap2d.getResolution(), gridMap.getResolution());
  EXPECT_EQ(costmap2d.getOriginX(), position.x());
  EXPECT_EQ(costmap2d.getOriginY(), position.y());
}

TEST(Costmap2DConversion, convertFrom)
{
  CostmapConverter<GridMap> costmapConverter;

  // Create Costmap2D.
  costmap_2d::Costmap2D costmap2d(8, 5, 1.0, 2.0, 3.0);

  // Create grid map.
  const std::string layer("layer");
  GridMap gridMap;
  costmapConverter.initializeFromCostmap2d(costmap2d, gridMap);

  // Fill in test data to Costmap2D.
  Position position1(8.5, 4.5);
  Position position2(3.2, 5.1);
  Position position3(5.2, 7.8);
  {
    unsigned int xIndex, yIndex;
    ASSERT_TRUE(costmap2d.worldToMap(position1.x(), position1.y(), xIndex, yIndex));
    costmap2d.getCharMap()[costmap2d.getIndex(xIndex, yIndex)] = 1;
  }
  {
    unsigned int xIndex, yIndex;
    ASSERT_TRUE(costmap2d.worldToMap(position2.x(), position2.y(), xIndex, yIndex));
    costmap2d.getCharMap()[costmap2d.getIndex(xIndex, yIndex)] = 254;
  }
  {
    unsigned int xIndex, yIndex;
    ASSERT_TRUE(costmap2d.worldToMap(position3.x(), position3.y(), xIndex, yIndex));
    costmap2d.getCharMap()[costmap2d.getIndex(xIndex, yIndex)] = 255;
  }

  // Copy data.
  costmapConverter.fromCostmap2d(costmap2d, layer, gridMap);

  // Check data.
  EXPECT_EQ(1.0, gridMap.atPosition(layer, position1));
  EXPECT_EQ(100.0, gridMap.atPosition(layer, position2));
  EXPECT_EQ(-1.0, gridMap.atPosition(layer, position3));
}
