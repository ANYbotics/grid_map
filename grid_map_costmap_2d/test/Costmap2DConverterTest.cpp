/*
 * Costmap2DConverterTest.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// Grid map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>

// Gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

using namespace grid_map;

TEST(Costmap2DConversion, initializeFromCostmap2d)
{
  Costmap2DConverter<GridMap> costmap2dConverter;
  // Create Costmap2D.
  costmap_2d::Costmap2D costmap2d(8, 5, 1.0, 2.0, 3.0);

  // Convert to grid map.
  GridMap gridMap;
  costmap2dConverter.initializeFromCostmap2D(costmap2d, gridMap);

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

TEST(Costmap2DConversion, addLayerFromCostmap2d)
{
  Costmap2DConverter<GridMap> costmap2dConverter;

  // Create Costmap2D.
  costmap_2d::Costmap2D costmap2d(8, 5, 1.0, 2.0, 3.0);

  // Create grid map.
  const std::string layer("layer");
  GridMap gridMap;
  costmap2dConverter.initializeFromCostmap2D(costmap2d, gridMap);

  // Set test data.
  using TestValue = std::tuple<Position, unsigned char, double>;
  std::vector<TestValue> testValues;
  testValues.push_back(TestValue(Position(8.5, 4.5), 1, 1.0));
  testValues.push_back(TestValue(Position(3.2, 5.1), 254, 100.0));
  testValues.push_back(TestValue(Position(5.2, 7.8), 255, -1.0));

  // Fill in test data to Costmap2D.
  for (const auto& testValue : testValues) {
    unsigned int xIndex, yIndex;
    ASSERT_TRUE(costmap2d.worldToMap(std::get<0>(testValue).x(), std::get<0>(testValue).y(), xIndex, yIndex));
    costmap2d.getCharMap()[costmap2d.getIndex(xIndex, yIndex)] = std::get<1>(testValue);
  }

  // Copy data.
  costmap2dConverter.addLayerFromCostmap2D(costmap2d, layer, gridMap);

  // Check data.
  for (const auto& testValue : testValues) {
    EXPECT_EQ(std::get<2>(testValue), gridMap.atPosition(layer, std::get<0>(testValue)));
  }
}
