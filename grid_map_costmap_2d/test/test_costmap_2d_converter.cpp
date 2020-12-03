/*
 * test_costmap_2d_converter.cpp
 *
 *  Created on: Nov 23, 2016
 *      Authors: Peter Fankhauser, Gabriel Hottiger
 *      Institute: ETH Zurich, ANYbotics
 */

// Gtest
#include <gtest/gtest.h>

#include <tuple>
#include <vector>
#include <string>

// Grid map
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_costmap_2d/grid_map_costmap_2d.hpp"

template<typename ConversionTable>
class TestCostmap2DConversion
  : public testing::Test
{
public:
  using TestValue = std::tuple<grid_map::Position, unsigned char, double, bool>;

  //! Constructor
  TestCostmap2DConversion()
  : costmap2d_(8, 5, 1.0, 2.0, 3.0)
  {
    gridMap_.setGeometry(grid_map::Length(8.0, 5.0), 1.0, grid_map::Position(6.0, 5.5));
  }

  //! Getter of test data
  std::vector<TestValue> getTestValues();

  //! Check that maps have same geometry
  void assertMapGeometry(
    const grid_map::GridMap & gridMap,
    const nav2_costmap_2d::Costmap2D & costMap)
  {
    // Check map info.
    // Different conventions: Costmap2d returns the *centerpoint* of the last cell in the map.
    grid_map::Length length =
      gridMap.getLength() - grid_map::Length::Constant(0.5 * gridMap.getResolution());
    grid_map::Length position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
    EXPECT_EQ(costMap.getSizeInMetersX(), length.x());
    EXPECT_EQ(costMap.getSizeInMetersY(), length.y());
    EXPECT_EQ(costMap.getSizeInCellsX(), (unsigned int) gridMap.getSize()[0]);
    EXPECT_EQ(costMap.getSizeInCellsY(), (unsigned int) gridMap.getSize()[1]);
    EXPECT_EQ(costMap.getResolution(), gridMap.getResolution());
    EXPECT_EQ(costMap.getOriginX(), position.x());
    EXPECT_EQ(costMap.getOriginY(), position.y());
  }

protected:
  grid_map::Costmap2DConverter<grid_map::GridMap, ConversionTable> costmap2dConverter_;
  nav2_costmap_2d::Costmap2D costmap2d_;
  grid_map::GridMap gridMap_;
};

//! Map type that has only mappings for cost map special values.
using Costmap2DSpecialTranslationTable = grid_map::Costmap2DTranslationTable<3, 2, 1, 0>;
//! Map type that has larger intervals between special values.
using Costmap2DLargeIntervalsTranslationTable =
  grid_map::Costmap2DTranslationTable<5, 500, 400, 100>;

// Test data for special translation
template<>
std::vector<TestCostmap2DConversion<Costmap2DSpecialTranslationTable>::TestValue>
TestCostmap2DConversion<Costmap2DSpecialTranslationTable>::getTestValues()
{
  std::vector<TestValue> testValues;
  testValues.emplace_back(
    TestValue(
      grid_map::Position(3.2, 5.1),
      nav2_costmap_2d::FREE_SPACE,
      0.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(4.2, 4.1),
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
      1.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(6.2, 3.1),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      2.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(5.2, 7.8),
      nav2_costmap_2d::NO_INFORMATION,
      3.f,
      true));
  // Check for grid map to costmap only.
  testValues.emplace_back(
    TestValue(
      grid_map::Position(5.4, 6.8),
      nav2_costmap_2d::FREE_SPACE,
      -1.f,
      false));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(8.5, 4.5),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      4.f,
      false));
  return testValues;
}

// Test data for special translation
template<>
std::vector<TestCostmap2DConversion<Costmap2DLargeIntervalsTranslationTable>::TestValue>
TestCostmap2DConversion<Costmap2DLargeIntervalsTranslationTable>::getTestValues()
{
  std::vector<TestValue> testValues;
  testValues.emplace_back(
    TestValue(
      grid_map::Position(3.2, 5.1),
      nav2_costmap_2d::FREE_SPACE,
      100.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(4.2, 4.1),
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
      400.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(6.2, 3.1),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      500.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(5.2, 7.8),
      nav2_costmap_2d::NO_INFORMATION,
      5.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(3.4, 3.8),
      253 / 11,
      100.f + 300.f / 11.f,
      true));
  // Check for grid map to costmap only.
  testValues.emplace_back(
    TestValue(
      grid_map::Position(5.4, 6.8),
      nav2_costmap_2d::FREE_SPACE,
      83.f,
      false));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(8.5, 4.5),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      504.f,
      false));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(4.7, 5.8),
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
      444.f,
      false));

  return testValues;
}

// Test data for direct translation
template<>
std::vector<TestCostmap2DConversion<grid_map::Costmap2DDirectTranslationTable>::TestValue>
TestCostmap2DConversion<grid_map::Costmap2DDirectTranslationTable>::getTestValues()
{
  std::vector<TestValue> testValues;
  testValues.emplace_back(
    TestValue(
      grid_map::Position(3.2, 5.1),
      nav2_costmap_2d::FREE_SPACE,
      nav2_costmap_2d::FREE_SPACE,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(4.2, 4.1),
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(6.2, 3.1),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      nav2_costmap_2d::LETHAL_OBSTACLE,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(5.2, 7.8),
      nav2_costmap_2d::NO_INFORMATION,
      nav2_costmap_2d::NO_INFORMATION,
      true));
  testValues.emplace_back(TestValue(grid_map::Position(5.4, 6.8), 1, 1.0, true));
  testValues.emplace_back(TestValue(grid_map::Position(8.5, 4.5), 97, 97.0, true));
  // Check for grid map to costmap only.
  testValues.emplace_back(
    TestValue(
      grid_map::Position(4.7, 5.8),
      nav2_costmap_2d::FREE_SPACE,
      -30.f,
      false));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(3.4, 3.8),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      270.f,
      false));
  return testValues;
}

// Test data for century translation
template<>
std::vector<TestCostmap2DConversion<grid_map::Costmap2DCenturyTranslationTable>::TestValue>
TestCostmap2DConversion<grid_map::Costmap2DCenturyTranslationTable>::getTestValues()
{
  std::vector<TestValue> testValues;
  testValues.emplace_back(
    TestValue(
      grid_map::Position(3.2, 5.1),
      nav2_costmap_2d::FREE_SPACE,
      0.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(4.2, 4.1),
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
      99.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(6.2, 3.1),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      100.f,
      true));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(5.2, 7.8),
      nav2_costmap_2d::NO_INFORMATION,
      -1.f,
      true));
  testValues.emplace_back(TestValue(grid_map::Position(5.4, 6.8), 253 / 11, 99.f / 11.f, true));
  // Check for grid map to costmap only.
  testValues.emplace_back(
    TestValue(
      grid_map::Position(4.7, 5.8),
      nav2_costmap_2d::FREE_SPACE,
      -30.f,
      false));
  testValues.emplace_back(
    TestValue(
      grid_map::Position(3.4, 3.8),
      nav2_costmap_2d::LETHAL_OBSTACLE,
      270.f,
      false));
  return testValues;
}

using TranslationTableTestTypes = ::testing::Types<
  Costmap2DSpecialTranslationTable,
  Costmap2DLargeIntervalsTranslationTable,
  grid_map::Costmap2DDirectTranslationTable,
  grid_map::Costmap2DCenturyTranslationTable>;

TYPED_TEST_SUITE(
  TestCostmap2DConversion, TranslationTableTestTypes, testing::internal::DefaultNameGenerator);

TYPED_TEST(TestCostmap2DConversion, initializeFromCostmap2d)
{
  // Convert to grid map.
  grid_map::GridMap gridMap;
  this->costmap2dConverter_.initializeFromCostmap2D(this->costmap2d_, gridMap);
  this->assertMapGeometry(gridMap, this->costmap2d_);
}

TYPED_TEST(TestCostmap2DConversion, initializeFromGridMap)
{
  // Convert to Costmap2D.
  nav2_costmap_2d::Costmap2D costmap2d;
  this->costmap2dConverter_.initializeFromGridMap(this->gridMap_, costmap2d);
  this->assertMapGeometry(this->gridMap_, costmap2d);
}

TYPED_TEST(TestCostmap2DConversion, addLayerFromCostmap2d)
{
  // Create grid map.
  const std::string layer("layer");
  grid_map::GridMap gridMap;
  this->costmap2dConverter_.initializeFromCostmap2D(this->costmap2d_, gridMap);

  // Fill in test data to Costmap2D.
  for (const auto & testValue : this->getTestValues()) {
    if (std::get<3>(testValue)) {
      unsigned int xIndex, yIndex;
      ASSERT_TRUE(
        this->costmap2d_.worldToMap(
          std::get<0>(testValue).x(),
          std::get<0>(testValue).y(),
          xIndex,
          yIndex));
      this->costmap2d_.getCharMap()[this->costmap2d_.getIndex(xIndex, yIndex)] =
        std::get<1>(testValue);
    }
  }

  // Copy data.
  this->costmap2dConverter_.addLayerFromCostmap2D(this->costmap2d_, layer, gridMap);

  // Check data.
  for (const auto & testValue : this->getTestValues()) {
    if (std::get<3>(testValue)) {
      EXPECT_EQ(std::get<2>(testValue), gridMap.atPosition(layer, std::get<0>(testValue)));
    }
  }
}

TYPED_TEST(TestCostmap2DConversion, setCostmap2DFromGridMap)
{
  // Create costmap2d.
  nav2_costmap_2d::Costmap2D costmap;
  this->costmap2dConverter_.initializeFromGridMap(this->gridMap_, costmap);

  // Fill in test data to grid map.
  const std::string layer("layer");
  this->gridMap_.add(layer);
  for (const auto & testValue : this->getTestValues()) {
    grid_map::Index index;
    this->gridMap_.getIndex(std::get<0>(testValue), index);
    this->gridMap_.get(layer)(index(0), index(1)) = std::get<2>(testValue);
  }

  // Copy data.
  this->costmap2dConverter_.setCostmap2DFromGridMap(this->gridMap_, layer, costmap);

  // Check data.
  for (const auto & testValue : this->getTestValues()) {
    unsigned int xIndex, yIndex;
    ASSERT_TRUE(
      costmap.worldToMap(
        std::get<0>(testValue).x(),
        std::get<0>(testValue).y(),
        xIndex,
        yIndex));
    costmap.getCharMap()[costmap.getIndex(xIndex, yIndex)] = std::get<1>(testValue);
  }
}
