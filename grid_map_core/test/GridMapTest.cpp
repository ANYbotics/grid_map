/*
 * GridMapTest.cpp
 *
 *  Created on: Aug 26, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

// Math
#include <math.h>

using namespace std;
using namespace grid_map;

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


TEST(AddDataFrom, extendMapAligned)
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

TEST(AddDataFrom, extendMapNotAligned)
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

TEST(AddDataFrom, copyData)
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
