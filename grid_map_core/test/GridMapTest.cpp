/*
 * GridMapTest.cpp
 *
 *  Created on: Aug 26, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

// Math
#include <math.h>

using namespace std;
using namespace grid_map;

TEST(GridMap, CopyConstructor)
{
  GridMap map({"layer_a", "layer_b"});
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);
  GridMap mapCopy(map);
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
  GridMap map({"layer_a", "layer_b"});
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);
  GridMap mapCopy;
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
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer", 0.0);
  map.setBasicLayers(map.getLayers());
  std::vector<BufferRegion> regions;
  map.move(Position(-3.0, -2.0), regions);
  Index startIndex = map.getStartIndex();

  EXPECT_EQ(3, startIndex(0));
  EXPECT_EQ(2, startIndex(1));

  EXPECT_FALSE(map.isValid(Index(0, 0))); // TODO Check entire map.
  EXPECT_TRUE(map.isValid(Index(3, 2)));
  EXPECT_FALSE(map.isValid(Index(2, 2)));
  EXPECT_FALSE(map.isValid(Index(3, 1)));
  EXPECT_TRUE(map.isValid(Index(7, 4)));

  EXPECT_EQ(2, regions.size());
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
  GridMap map;
  const auto heightLayerName = "height";

  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.setBasicLayers(map.getLayers());
  map.get(heightLayerName)(0,0) = 1.0;

  // Transformation (90° rotation).
  Eigen::Isometry3d transform;

  transform.translation().x() = 0.0;
  transform.translation().y() = 0.0;
  transform.translation().z() = 0.0;

  transform.linear()(0,0) =  0.0;
  transform.linear()(0,1) = -1.0;
  transform.linear()(0,2) =  0.0;

  transform.linear()(1,0) =  1.0;
  transform.linear()(1,1) =  0.0;
  transform.linear()(1,2) =  0.0;

  transform.linear()(2,0) =  0.0;
  transform.linear()(2,1) =  0.0;
  transform.linear()(2,2) =  1.0;

  // Apply affine transformation.
  const GridMap transformedMap = map.getTransformedMap(transform, heightLayerName, map.getFrameId(), 0.25);

  // Check if map has been rotated by 90° about z
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().y(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().x(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());
  EXPECT_DOUBLE_EQ(map.get(heightLayerName)(0,0), transformedMap.get(heightLayerName)(19,0));
}

TEST(GridMap, ClipToMap)
{
  GridMap map({"layer_a", "layer_b"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.5, 0.5));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);

  const Position positionInMap = Position(0.4, 0.3); // position located inside the map
  const Position positionOutMap = Position(1.0, 2.0); // position located outside the map

  const Position clippedPositionInMap = map.getClosestPositionInMap(positionInMap);
  const Position clippedPositionOutMap = map.getClosestPositionInMap(positionOutMap);

  // Check if position-in-map remains unchanged.
  EXPECT_NEAR(clippedPositionInMap.x(),positionInMap.x(), 1e-6);
  EXPECT_NEAR(clippedPositionInMap.y(), positionInMap.y(), 1e-6);

  // Check if position-out-map is indeed outside of the map.
  EXPECT_TRUE(!map.isInside(positionOutMap));

  // Check if position-out-map has been projected into the map.
  EXPECT_TRUE(map.isInside(clippedPositionOutMap));
}



TEST(GridMap, ClipToMap2)
{
  GridMap map({"types"});
  map.setGeometry(Length(1.0, 1.0), 0.05, Position(0.0, 0.0));

  // Test 8 points outside of map.
  /*
   * A  B  C
   *  +---+
   *  |   |         X
   * D|   |E        ^
   *  |   |         |
   *  +---+     Y<--+
   * F  G  H
   *
   * Note: Position to index alignment is an half open interval.
   *       An example position of 0.5 is assigned to the upper index.
   *       The interval in the current example is: 
   *       Position: [...)[0.485 ... 0.5)[0.5 ... 0.505)[...)
   *       Index:      8          9           10          11
   */

  Index insideIndex;
  Position outsidePosition;

  // Point A
  outsidePosition = Position(1.0, 1.0);
  auto closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  bool isInside = map.getIndex(closestInsidePosition, insideIndex);

  auto expectedPosition = Position(0.5, 0.5);
  auto expectedIndex = Index(0, 0);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point B
  outsidePosition = Position(1.0, 0.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  isInside = map.getIndex(closestInsidePosition, insideIndex);

  expectedPosition = Position(0.5, 0.0);
  expectedIndex = Index(0, 10);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point C
  outsidePosition = Position(1.0, -1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  isInside = map.getIndex(closestInsidePosition, insideIndex);

  expectedPosition = Position(0.5, -0.5);
  expectedIndex = Index(0, 19);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point D
  outsidePosition = Position(0.0, 1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  isInside = map.getIndex(closestInsidePosition, insideIndex);

  expectedPosition = Position(0.0, 0.5);
  expectedIndex = Index(10, 0);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point E
  outsidePosition = Position(0.0, -1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  isInside = map.getIndex(closestInsidePosition, insideIndex);

  expectedPosition = Position(0.0, -0.5);
  expectedIndex = Index(10, 19);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point F
  outsidePosition = Position(-1.0, 1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  isInside = map.getIndex(closestInsidePosition, insideIndex);

  expectedPosition = Position(-0.5, 0.5);
  expectedIndex = Index(19, 0);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point G
  outsidePosition = Position(-1.0, 0.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  isInside = map.getIndex(closestInsidePosition, insideIndex);

  expectedPosition = Position(-0.5, 0.0);
  expectedIndex = Index(19, 10);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point H
  outsidePosition = Position(-1.0, -1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  isInside = map.getIndex(closestInsidePosition, insideIndex);

  expectedPosition = Position(-0.5, -0.5);
  expectedIndex = Index(19, 19);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;
}

TEST(AddDataFrom, ExtendMapAligned)
{
  GridMap map1, map2;
  map1.setGeometry(Length(5.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(5, 5)
  map1.add("zero", 0.0);
  map1.add("one", 1.0);
  map1.setBasicLayers(map1.getLayers());

  map2.setGeometry(Length(3.1, 3.1), 1.0, Position(2.0, 2.0));
  map2.add("one", 1.1);
  map2.add("two", 2.0);
  map2.setBasicLayers(map1.getLayers());

  map1.addDataFrom(map2, true, true, true);

  EXPECT_TRUE(map1.exists("two"));
  EXPECT_TRUE(map1.isInside(Position(3.0, 3.0)));
  EXPECT_DOUBLE_EQ(6.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(6.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(0.5, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(0.5, map1.getPosition().y());
  EXPECT_NEAR(1.1, map1.atPosition("one", Position(2, 2)), 1e-4);
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("one", Position(-2, -2)));
  EXPECT_DOUBLE_EQ(0.0, map1.atPosition("zero", Position(0.0, 0.0)));
}

TEST(AddDataFrom, ExtendMapNotAligned)
{
  GridMap map1, map2;
  map1.setGeometry(Length(6.1, 6.1), 1.0, Position(0.0, 0.0)); // bufferSize(6, 6)
  map1.add("nan");
  map1.add("one", 1.0);
  map1.add("zero", 0.0);
  map1.setBasicLayers(map1.getLayers());

  map2.setGeometry(Length(3.1, 3.1), 1.0, Position(3.2, 3.2));
  map2.add("nan", 1.0);
  map2.add("one", 1.1);
  map2.add("two", 2.0);
  map2.setBasicLayers(map1.getLayers());

  std::vector<std::string> stringVector;
  stringVector.push_back("nan");
  map1.addDataFrom(map2, true, false, false, stringVector);
  Index index;
  map1.getIndex(Position(-2, -2), index);

  EXPECT_FALSE(map1.exists("two"));
  EXPECT_TRUE(map1.isInside(Position(4.0, 4.0)));
  EXPECT_DOUBLE_EQ(8.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(8.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(1.0, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(1.0, map1.getPosition().y());
  EXPECT_FALSE(map1.isValid(index, "nan"));
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("one", Position(0.0, 0.0)));
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("nan", Position(3.0, 3.0)));
}

TEST(AddDataFrom, CopyData)
{
  GridMap map1, map2;
  map1.setGeometry(Length(5.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(5, 5)
  map1.add("zero", 0.0);
  map1.add("one");
  map1.setBasicLayers(map1.getLayers());

  map2.setGeometry(Length(3.1, 3.1), 1.0, Position(2.0, 2.0));
  map2.add("one", 1.0);
  map2.add("two", 2.0);
  map2.setBasicLayers(map1.getLayers());

  map1.addDataFrom(map2, false, false, true);
  Index index;
  map1.getIndex(Position(-2, -2), index);

  EXPECT_TRUE(map1.exists("two"));
  EXPECT_FALSE(map1.isInside(Position(3.0, 3.0)));
  EXPECT_DOUBLE_EQ(5.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(5.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(0.0, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(0.0, map1.getPosition().y());
  EXPECT_DOUBLE_EQ(1.0, map1.atPosition("one", Position(2, 2)));
  EXPECT_FALSE(map1.isValid(index, "one"));
  EXPECT_DOUBLE_EQ(0.0, map1.atPosition("zero", Position(0.0, 0.0)));
}

TEST(ValueAtPosition, NearestNeighbor)
{
  GridMap map( { "types" });
  map.setGeometry(Length(3.0, 3.0), 1.0, Position(0.0, 0.0));

  map.at("types", Index(0,0)) = 0.5;
  map.at("types", Index(0,1)) = 3.8;
  map.at("types", Index(0,2)) = 2.0;
  map.at("types", Index(1,0)) = 2.1;
  map.at("types", Index(1,1)) = 1.0;
  map.at("types", Index(1,2)) = 2.0;
  map.at("types", Index(2,0)) = 1.0;
  map.at("types", Index(2,1)) = 2.0;
  map.at("types", Index(2,2)) = 2.0;

  double value;

  value = map.atPosition("types", Position(1.35,-0.4));
  EXPECT_DOUBLE_EQ((float)3.8, value);

  value = map.atPosition("types", Position(-0.3,0.0));
  EXPECT_DOUBLE_EQ(1.0, value);
}

TEST(ValueAtPosition, LinearInterpolated)
{
  GridMap map( { "types" });
  map.setGeometry(Length(3.0, 3.0), 1.0, Position(0.0, 0.0));

  map.at("types", Index(0,0)) = 0.5;
  map.at("types", Index(0,1)) = 3.8;
  map.at("types", Index(0,2)) = 2.0;
  map.at("types", Index(1,0)) = 2.1;
  map.at("types", Index(1,1)) = 1.0;
  map.at("types", Index(1,2)) = 2.0;
  map.at("types", Index(2,0)) = 1.0;
  map.at("types", Index(2,1)) = 2.0;
  map.at("types", Index(2,2)) = 2.0;

  double value;

  // Close to the border -> reverting to INTER_NEAREST.
  value = map.atPosition("types", Position(-0.5,-1.2), InterpolationMethods::INTER_LINEAR);
  EXPECT_DOUBLE_EQ(2.0, value);
  // In between 1.0 and 2.0 field.
  value = map.atPosition("types", Position(-0.5,0.0), InterpolationMethods::INTER_LINEAR);
  EXPECT_DOUBLE_EQ(1.5, value);
  // Calculated "by Hand".
  value = map.atPosition("types", Position(0.69,0.38), InterpolationMethods::INTER_LINEAR);
  EXPECT_NEAR(2.1963200, value, 0.0000001);
}
