/*
 * SignedDistanceFieldTest.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: Takahiro Miki
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_sdf/SignedDistanceField.hpp"

// gtest
#include <gtest/gtest.h>

// Math
#include <math.h>

using namespace std;
using namespace grid_map;
// using namespace Eigen;
//
TEST(SignedDistanceField, GetDistanceFlat)
{
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  map["layer"].setConstant(1.0);
  map.at("layer", Index(0,0)) = -1.0;

  grid_map_sdf::SignedDistanceField sdf;
  sdf.calculateSignedDistanceField(map, "layer", 2.5);
  Position pos;
  map.getPosition(Index(9, 9), pos);
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.1)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.2)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.3)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.4)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.6)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.7)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.8)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.9)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  // cout << sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.9)) << endl;
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)), -1.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.1)), -0.9, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.2)), -0.8, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.3)), -0.7, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.4)), -0.6, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)), -0.5, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.6)), -0.4, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.7)), -0.3, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.8)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.9)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)), 0.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)), 0.2, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)), 0.3, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)), 0.4, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)), 0.5, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)), 0.6, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)), 0.7, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.8, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.9)), 0.9, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.0)), 1.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.1)), 1.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.2)), 1.2, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.3)), 1.3, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.4)), 1.4, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.5)), 1.5, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 10.0)), 2.5, 0.0001);
}


TEST(SignedDistanceField, GetDistance)
{
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.15, 0.25));
  map["layer"].setConstant(1.0);

  map.at("layer", Index(3,4)) = 2.0;
  map.at("layer", Index(3,5)) = 2.0;
  map.at("layer", Index(3,6)) = 2.0;
  map.at("layer", Index(4,4)) = 2.0;
  map.at("layer", Index(4,5)) = 2.0;
  map.at("layer", Index(4,6)) = 2.0;
  map.at("layer", Index(5,4)) = 2.0;
  map.at("layer", Index(5,5)) = 2.0;
  map.at("layer", Index(5,6)) = 2.0;
  map.at("layer", Index(6,4)) = 2.0;
  map.at("layer", Index(6,5)) = 2.0;
  map.at("layer", Index(6,6)) = 2.0;
  map.at("layer", Index(7,4)) = 2.0;
  map.at("layer", Index(7,5)) = 2.0;
  map.at("layer", Index(7,6)) = 2.0;

  grid_map_sdf::SignedDistanceField sdf;
  sdf.calculateSignedDistanceField(map, "layer", 1.5);
  Position pos;

  map.getPosition(Index(5, 5), pos);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)), -1.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)), -1.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)), -1.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.0)), 0.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.1)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.2)), 0.2, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.3)), 0.3, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.4)), 0.4, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.5)), 0.5, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 10.0)), 1.5, 0.0001);

  map.getPosition(Index(5, 2), pos);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)), 0.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)), 0.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)), 0.0, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.9)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.0)), 0.2, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.1)), 0.3, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.2)), 0.4, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.3)), 0.5, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.4)), 0.6, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.5)), 0.7, 0.0001);
  EXPECT_NEAR(sdf.getDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 10.0)), 1.7, 0.0001);
}

TEST(SignedDistanceField, GetInterpolatedDistance){
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.15, 0.25));
  map["layer"].setConstant(1.0);

  map.at("layer", Index(3,3)) = 2.0;
  map.at("layer", Index(3,4)) = 2.0;
  map.at("layer", Index(3,5)) = 2.0;
  map.at("layer", Index(3,6)) = 2.0;
  map.at("layer", Index(3,7)) = 2.0;
  map.at("layer", Index(4,3)) = 2.0;
  map.at("layer", Index(4,4)) = 2.0;
  map.at("layer", Index(4,5)) = 2.0;
  map.at("layer", Index(4,6)) = 2.0;
  map.at("layer", Index(4,7)) = 2.0;
  map.at("layer", Index(5,3)) = 2.0;
  map.at("layer", Index(5,4)) = 2.0;
  map.at("layer", Index(5,5)) = 2.0;
  map.at("layer", Index(5,6)) = 2.0;
  map.at("layer", Index(5,7)) = 2.0;
  map.at("layer", Index(6,3)) = 2.0;
  map.at("layer", Index(6,4)) = 2.0;
  map.at("layer", Index(6,5)) = 2.0;
  map.at("layer", Index(6,6)) = 2.0;
  map.at("layer", Index(6,7)) = 2.0;
  map.at("layer", Index(7,3)) = 2.0;
  map.at("layer", Index(7,4)) = 2.0;
  map.at("layer", Index(7,5)) = 2.0;
  map.at("layer", Index(7,6)) = 2.0;
  map.at("layer", Index(7,7)) = 2.0;
  // map.at("layer", Index(0,0)) = -2.0;

  grid_map_sdf::SignedDistanceField sdf;
  sdf.calculateSignedDistanceField(map, "layer", 1.5);
  // cout << sdf.getMapPosition() << endl;;
  // cout << sdf.getMapSize() << endl;;
  cout << sdf.getMapResolution() << endl;;
  Position pos;
  map.getPosition(Index(5, 5), pos);
  cout << "pos = " << pos << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.1)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.2)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.3)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.4)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.6)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.7)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.8)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.9)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.9)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.0)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.1)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.2)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.3)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.4)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.5)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.6)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.7)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.8)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.9)) << endl;
  cout << "--------------------------------" << endl;
  map.getPosition(Index(5, 4), pos);
  cout << "pos = " << pos << endl;
  map.getPosition(Index(5, 1), pos);
  cout << "pos = " << pos << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.1)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.2)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.3)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.4)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.6)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.7)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.8)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.9)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.9)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.0)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.1)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.2)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.3)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.4)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.5)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.6)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.7)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.8)) << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.9)) << endl;
  cout << "--------------------------------" << endl;
  map.getPosition(Index(5, 0), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 1), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 2), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 3), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 4), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 5), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 6), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 7), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 8), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;
  map.getPosition(Index(5, 9), pos);
  cout << "pos = " << pos << endl;
  cout << "d = " << endl;
  cout << sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)) << endl;

  map.getPosition(Index(5, 5), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)), -1.0, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)), -1.0, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)), -1.0, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), -0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.9)), -0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.0)), 0.0, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.1)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.2)), 0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.3)), 0.3, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.4)), 0.4, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.5)), 0.5, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 10.0)), 1.5, 0.0001);

  map.getPosition(Index(5, 1), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.0)), -0.25, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 0.5)), -0.25, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.0)), -0.25, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.1)), 0.05, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.2)), 0.125, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.3)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.4)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.5)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.6)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.7)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.9)), 0.1, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.0)), 0.2, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.1)), 0.3, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.2)), 0.4, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.3)), 0.5, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.4)), 0.6, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 2.5)), 0.7, 0.0001);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 10.0)), 1.7, 0.0001);

  map.getPosition(Index(5, 0), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.275, 0.0001);
  map.getPosition(Index(5, 1), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.1, 0.0001);
  map.getPosition(Index(5, 2), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.0, 0.0001);
  map.getPosition(Index(5, 3), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), -0.15, 0.0001);
  map.getPosition(Index(5, 4), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), -0.2, 0.0001);
  map.getPosition(Index(5, 5), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), -0.2, 0.0001);
  map.getPosition(Index(5, 6), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), -0.1, 0.0001);
  map.getPosition(Index(5, 7), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.025, 0.0001);
  map.getPosition(Index(5, 8), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.125, 0.0001);
  map.getPosition(Index(5, 9), pos);
  EXPECT_NEAR(sdf.getInterpolatedDistanceAt(Eigen::Vector3f(pos.x(), pos.y(), 1.8)), 0.25, 0.0001);
}
