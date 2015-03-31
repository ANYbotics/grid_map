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
using namespace grid_map_lib;
using namespace grid_map;

//TEST(checkSortVertices, checkSortVertices)
//{
//  grid_map::Polygon polygon;
//  polygon.addVertex(Vector2d(-100.0, 100.0));
//  polygon.addVertex(Vector2d(100.0, 100.0));
//  polygon.addVertex(Vector2d(100.0, -100.0));
//  polygon.addVertex(Vector2d(-100.0, -100.0));
//
//  EXPECT_TRUE(polygon.sortVertices(polygon.getVertex(0), polygon.getVertex(1)));
//  EXPECT_FALSE(polygon.sortVertices(polygon.getVertex(1), polygon.getVertex(2)));
//  EXPECT_TRUE(polygon.sortVertices(polygon.getVertex(3), polygon.getVertex(0)));
//  EXPECT_FALSE(polygon.sortVertices(polygon.getVertex(2), polygon.getVertex(3)));
//}
//
//TEST(computeCrossProduct2D, computeCrossProduct2D)
//{
//  grid_map::Polygon polygon;
//  polygon.addVertex(Vector2d(1.0, 1.0));
//  polygon.addVertex(Vector2d(2.0, 1.0));
//  polygon.addVertex(Vector2d(3.0, 1.0));
//  polygon.addVertex(Vector2d(4.0, 1.0));
//
//  EXPECT_DOUBLE_EQ(-1.0, polygon.computeCrossProduct2D(polygon.getVertex(0), polygon.getVertex(1)));
//  EXPECT_DOUBLE_EQ(-1.0, polygon.computeCrossProduct2D(polygon.getVertex(1), polygon.getVertex(2)));
//  EXPECT_DOUBLE_EQ(0.0, polygon.computeCrossProduct2D(polygon.getVertex(0), polygon.getVertex(0)));
//  EXPECT_DOUBLE_EQ(1.0, polygon.computeCrossProduct2D(polygon.getVertex(3), polygon.getVertex(2)));
//}

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
