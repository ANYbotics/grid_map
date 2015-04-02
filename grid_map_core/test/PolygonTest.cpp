/*
 * GridMapTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/Polygon.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace grid_map;

TEST(checkConvexHull, createHull)
{
  grid_map::Polygon polygon1;
  polygon1.addVertex(Vector2d(0.0, 0.0));
  polygon1.addVertex(Vector2d(1.0, 1.0));
  polygon1.addVertex(Vector2d(0.0, 1.0));
  polygon1.addVertex(Vector2d(1.0, 0.0));

  grid_map::Polygon polygon2;
  polygon2.addVertex(Vector2d(0.5, 0.5));
  polygon2.addVertex(Vector2d(0.5, 1.5));
  polygon2.addVertex(Vector2d(1.5, 0.5));
  polygon2.addVertex(Vector2d(1.5, 1.5));

  grid_map::Polygon hull;
  hull = hull.convexHull(polygon1, polygon2);


  EXPECT_EQ(6, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.5)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.01, 1.49)));
}

TEST(checkConvexHullCircles, createHull)
{
  Position center1(0.0, 0.0);
  Position center2(1.0, 0.0);
  double radius = 0.5;

  grid_map::Polygon hull;
  hull = hull.convexHullCircles(center1, center2, radius);


  EXPECT_EQ(20, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.6)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.5, 0.2)));
}
