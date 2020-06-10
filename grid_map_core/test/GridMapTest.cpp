/*
 * GridMapTest.cpp
 *
 *  Created on: Aug 26, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// gtest
#include <gtest/gtest.h>

// Math
#include <math.h>

#include <vector>
#include <string>

#include "grid_map_core/GridMap.hpp"


TEST(GridMap, CopyConstructor)
{
  grid_map::GridMap map({"layer_a", "layer_b"});
  map.setGeometry(grid_map::Length(1.0, 2.0), 0.1, grid_map::Position(0.1, 0.2));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);
  grid_map::GridMap mapCopy(map);
  EXPECT_EQ(map.getSize()[0], mapCopy.getSize()[0]);
  EXPECT_EQ(map.getSize()[1], mapCopy.getSize()[1]);
  EXPECT_EQ(map.getLength().x(), mapCopy.getLength().x());
  EXPECT_EQ(map.getLength().y(), mapCopy.getLength().y());
  EXPECT_EQ(map.getPosition().x(), mapCopy.getPosition().x());
  EXPECT_EQ(map.getPosition().y(), mapCopy.getPosition().y());
  EXPECT_EQ(map.getLayers().size(), mapCopy.getLayers().size());
  EXPECT_EQ(map["layer_a"](0, 0), mapCopy["layer_a"](0, 0));
  EXPECT_EQ(map["layer_b"](0, 0), mapCopy["layer_b"](0, 0));
}

TEST(GridMap, CopyAssign)
{
  grid_map::GridMap map({"layer_a", "layer_b"});
  map.setGeometry(grid_map::Length(1.0, 2.0), 0.1, grid_map::Position(0.1, 0.2));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);
  grid_map::GridMap mapCopy;
  mapCopy = map;
  EXPECT_EQ(map.getSize()[0], mapCopy.getSize()[0]);
  EXPECT_EQ(map.getSize()[1], mapCopy.getSize()[1]);
  EXPECT_EQ(map.getLength().x(), mapCopy.getLength().x());
  EXPECT_EQ(map.getLength().y(), mapCopy.getLength().y());
  EXPECT_EQ(map.getPosition().x(), mapCopy.getPosition().x());
  EXPECT_EQ(map.getPosition().y(), mapCopy.getPosition().y());
  EXPECT_EQ(map.getLayers().size(), mapCopy.getLayers().size());
  EXPECT_EQ(map["layer_a"](0, 0), mapCopy["layer_a"](0, 0));
  EXPECT_EQ(map["layer_b"](0, 0), mapCopy["layer_b"](0, 0));
}

TEST(GridMap, Move)
{
  grid_map::GridMap map;
  // bufferSize(8, 5)
  map.setGeometry(grid_map::Length(8.1, 5.1), 1.0, grid_map::Position(0.0, 0.0));
  map.add("layer", 0.0);
  map.setBasicLayers(map.getLayers());
  std::vector<grid_map::BufferRegion> regions;
  map.move(grid_map::Position(-3.0, -2.0), regions);
  grid_map::Index startIndex = map.getStartIndex();

  EXPECT_EQ(3, startIndex(0));
  EXPECT_EQ(2, startIndex(1));

  EXPECT_FALSE(map.isValid(grid_map::Index(0, 0)));  // TODO(needs_assignment): Check entire map.
  EXPECT_TRUE(map.isValid(grid_map::Index(3, 2)));
  EXPECT_FALSE(map.isValid(grid_map::Index(2, 2)));
  EXPECT_FALSE(map.isValid(grid_map::Index(3, 1)));
  EXPECT_TRUE(map.isValid(grid_map::Index(7, 4)));

  EXPECT_EQ(2u, regions.size());
  EXPECT_EQ(0, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(3, regions[0].getSize()[0]);
  EXPECT_EQ(5, regions[0].getSize()[1]);
  EXPECT_EQ(0, regions[1].getStartIndex()[0]);
  EXPECT_EQ(0, regions[1].getStartIndex()[1]);
  EXPECT_EQ(8, regions[1].getSize()[0]);
  EXPECT_EQ(2, regions[1].getSize()[1]);
}

TEST(GridMap, Transform)
{
  // Initial map.
  grid_map::GridMap map;
  const auto heightLayerName = "height";

  map.setGeometry(grid_map::Length(1.0, 2.0), 0.1, grid_map::Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());
  map.get(heightLayerName)(0, 0) = 1.0;

  // Transformation (90° rotation).
  Eigen::Isometry3d transform;

  transform.translation().x() = 0.0;
  transform.translation().y() = 0.0;
  transform.translation().z() = 0.0;

  transform.linear()(0, 0) = 0.0;
  transform.linear()(0, 1) = -1.0;
  transform.linear()(0, 2) = 0.0;

  transform.linear()(1, 0) = 1.0;
  transform.linear()(1, 1) = 0.0;
  transform.linear()(1, 2) = 0.0;

  transform.linear()(2, 0) = 0.0;
  transform.linear()(2, 1) = 0.0;
  transform.linear()(2, 2) = 1.0;

  // Apply affine transformation.
  const grid_map::GridMap transformedMap = map.getTransformedMap(
    transform, heightLayerName,
    map.getFrameId(), 0.25);

  // Check if map has been rotated by 90° about z
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().y(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().x(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());
  EXPECT_DOUBLE_EQ(map.get(heightLayerName)(0, 0), transformedMap.get(heightLayerName)(19, 0));
}

TEST(GridMap, ClipToMap)
{
  grid_map::GridMap map({"layer_a", "layer_b"});
  map.setGeometry(grid_map::Length(1.0, 1.0), 0.1, grid_map::Position(0.5, 0.5));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);

  // position located outside the map
  const grid_map::Position positionInMap = grid_map::Position(0.4, 0.3);
  // position located outside the map
  const grid_map::Position positionOutMap = grid_map::Position(1.0, 2.0);

  const grid_map::Position clippedPositionInMap = map.getClosestPositionInMap(positionInMap);
  const grid_map::Position clippedPositionOutMap = map.getClosestPositionInMap(positionOutMap);

  // Check if position-in-map remains unchanged.
  EXPECT_NEAR(clippedPositionInMap.x(), positionInMap.x(), 1e-6);
  EXPECT_NEAR(clippedPositionInMap.y(), positionInMap.y(), 1e-6);

  // Check if position-out-map is indeed outside of the map.
  EXPECT_TRUE(!map.isInside(positionOutMap));

  // Check if position-out-map has been projected into the map.
  EXPECT_TRUE(map.isInside(clippedPositionOutMap));
}

TEST(AddDataFrom, ExtendMapAligned)
{
  grid_map::GridMap map1, map2;
  // bufferSize(5, 5)
  map1.setGeometry(grid_map::Length(5.1, 5.1), 1.0, grid_map::Position(0.0, 0.0));
  map1.add("zero", 0.0);
  map1.add("one", 1.0);
  map1.setBasicLayers(map1.getLayers());

  map2.setGeometry(grid_map::Length(3.1, 3.1), 1.0, grid_map::Position(2.0, 2.0));
  map2.add("one", 1.1);
  map2.add("two", 2.0);
  map2.setBasicLayers(map1.getLayers());

  map1.addDataFrom(map2, true, true, true);

  EXPECT_TRUE(map1.exists("two"));
  EXPECT_TRUE(map1.isInside(grid_map::Position(3.0, 3.0)));
  EXPECT_DOUBLE_EQ(6.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(6.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(0.5, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(0.5, map1.getPosition().y());
  EXPECT_NEAR(1.1, map1.atPosition("one", grid_map::Position(2, 2)), 1e-4);
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("one", grid_map::Position(-2, -2)));
  EXPECT_DOUBLE_EQ(0.0, map1.atPosition("zero", grid_map::Position(0.0, 0.0)));
}

TEST(AddDataFrom, ExtendMapNotAligned)
{
  grid_map::GridMap map1, map2;
  // bufferSize(6, 6)
  map1.setGeometry(grid_map::Length(6.1, 6.1), 1.0, grid_map::Position(0.0, 0.0));
  map1.add("nan");
  map1.add("one", 1.0);
  map1.add("zero", 0.0);
  map1.setBasicLayers(map1.getLayers());

  map2.setGeometry(grid_map::Length(3.1, 3.1), 1.0, grid_map::Position(3.2, 3.2));
  map2.add("nan", 1.0);
  map2.add("one", 1.1);
  map2.add("two", 2.0);
  map2.setBasicLayers(map1.getLayers());

  std::vector<std::string> stringVector;
  stringVector.push_back("nan");
  map1.addDataFrom(map2, true, false, false, stringVector);
  grid_map::Index index;
  map1.getIndex(grid_map::Position(-2, -2), index);

  EXPECT_FALSE(map1.exists("two"));
  EXPECT_TRUE(map1.isInside(grid_map::Position(4.0, 4.0)));
  EXPECT_DOUBLE_EQ(8.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(8.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(1.0, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(1.0, map1.getPosition().y());
  EXPECT_FALSE(map1.isValid(index, "nan"));
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("one", grid_map::Position(0.0, 0.0)));
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("nan", grid_map::Position(3.0, 3.0)));
}

TEST(AddDataFrom, CopyData)
{
  grid_map::GridMap map1, map2;
  // bufferSize(5, 5)
  map1.setGeometry(grid_map::Length(5.1, 5.1), 1.0, grid_map::Position(0.0, 0.0));
  map1.add("zero", 0.0);
  map1.add("one");
  map1.setBasicLayers(map1.getLayers());

  map2.setGeometry(grid_map::Length(3.1, 3.1), 1.0, grid_map::Position(2.0, 2.0));
  map2.add("one", 1.0);
  map2.add("two", 2.0);
  map2.setBasicLayers(map1.getLayers());

  map1.addDataFrom(map2, false, false, true);
  grid_map::Index index;
  map1.getIndex(grid_map::Position(-2, -2), index);

  EXPECT_TRUE(map1.exists("two"));
  EXPECT_FALSE(map1.isInside(grid_map::Position(3.0, 3.0)));
  EXPECT_DOUBLE_EQ(5.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(5.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(0.0, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(0.0, map1.getPosition().y());
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("one", grid_map::Position(2, 2)));
  EXPECT_FALSE(map1.isValid(index, "one"));
  EXPECT_DOUBLE_EQ(0.0, map1.atPosition("zero", grid_map::Position(0.0, 0.0)));
}

TEST(ValueAtPosition, NearestNeighbor)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(3.0, 3.0), 1.0, grid_map::Position(0.0, 0.0));

  map.at("types", grid_map::Index(0, 0)) = 0.5;
  map.at("types", grid_map::Index(0, 1)) = 3.8;
  map.at("types", grid_map::Index(0, 2)) = 2.0;
  map.at("types", grid_map::Index(1, 0)) = 2.1;
  map.at("types", grid_map::Index(1, 1)) = 1.0;
  map.at("types", grid_map::Index(1, 2)) = 2.0;
  map.at("types", grid_map::Index(2, 0)) = 1.0;
  map.at("types", grid_map::Index(2, 1)) = 2.0;
  map.at("types", grid_map::Index(2, 2)) = 2.0;

  double value;

  value = map.atPosition("types", grid_map::Position(1.35, -0.4));
  EXPECT_DOUBLE_EQ((float)3.8, value);

  value = map.atPosition("types", grid_map::Position(-0.3, 0.0));
  EXPECT_DOUBLE_EQ(1.0, value);
}

TEST(ValueAtPosition, LinearInterpolated)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(3.0, 3.0), 1.0, grid_map::Position(0.0, 0.0));

  map.at("types", grid_map::Index(0, 0)) = 0.5;
  map.at("types", grid_map::Index(0, 1)) = 3.8;
  map.at("types", grid_map::Index(0, 2)) = 2.0;
  map.at("types", grid_map::Index(1, 0)) = 2.1;
  map.at("types", grid_map::Index(1, 1)) = 1.0;
  map.at("types", grid_map::Index(1, 2)) = 2.0;
  map.at("types", grid_map::Index(2, 0)) = 1.0;
  map.at("types", grid_map::Index(2, 1)) = 2.0;
  map.at("types", grid_map::Index(2, 2)) = 2.0;

  double value;

  // Close to the border -> reverting to INTER_NEAREST.
  value = map.atPosition(
    "types", grid_map::Position(
      -0.5,
      -1.2),
    grid_map::InterpolationMethods::INTER_LINEAR);
  EXPECT_DOUBLE_EQ(2.0, value);
  // In between 1.0 and 2.0 field.
  value = map.atPosition(
    "types", grid_map::Position(
      -0.5,
      0.0),
    grid_map::InterpolationMethods::INTER_LINEAR);
  EXPECT_DOUBLE_EQ(1.5, value);
  // Calculated "by Hand".
  value = map.atPosition(
    "types", grid_map::Position(
      0.69,
      0.38),
    grid_map::InterpolationMethods::INTER_LINEAR);
  EXPECT_NEAR(2.1963200, value, 0.0000001);
}
