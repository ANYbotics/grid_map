/*
 * OctomapConverterTest.cpp
 *
 *  Created on: May 1, 2017
 *      Author: Jeff Delmerico
 *	 Institute: University of Zürich, Robotics and Perception Group
 */

// Grid map
#include <grid_map_core/GridMap.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>

// Octomap
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

// Gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

using namespace grid_map;

TEST(OctomapConversion, convertOctomapToGridMap)
{
  GridMapOctomapConverter octomapConverter;

  // Generate Octomap (a 1m sphere centered at (1,1,1))
  // Adapted from octomap unit_tests.cpp
  octomap::OcTree octomap(0.05);
  octomap::Pointcloud* measurement = new octomap::Pointcloud();

  octomap::point3d origin (1.0f, 1.0f, 1.0f);
  octomap::point3d point_on_surface (1.0f, 0.0f, 0.0f);

  for (int i=0; i<360; i++) {
    for (int j=0; j<360; j++) {
      octomap::point3d p = origin+point_on_surface;
      measurement->push_back(p);
      point_on_surface.rotate_IP (0,0,DEG2RAD(1.));
    }
    point_on_surface.rotate_IP (0,DEG2RAD(1.),0);
  }
  octomap.insertPointCloud(*measurement, origin);

  // Convert to grid map.
  GridMap gridMap;
  bool res = octomapConverter.fromOctomap(octomap, gridMap);
  ASSERT_TRUE(res);

  // Check map info.
  // Different conventions: Costmap2d returns the *centerpoint* of the last cell in the map.
  /*
  Length length = gridMap.getLength() - Length::Constant(0.5 * gridMap.getResolution());
  Length position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  EXPECT_EQ(costmap2d.getSizeInMetersX(), length.x());
  EXPECT_EQ(costmap2d.getSizeInMetersY(), length.y());
  EXPECT_EQ(costmap2d.getSizeInCellsX(), gridMap.getSize()[0]);
  EXPECT_EQ(costmap2d.getSizeInCellsY(), gridMap.getSize()[1]);
  EXPECT_EQ(costmap2d.getResolution(), gridMap.getResolution());
  EXPECT_EQ(costmap2d.getOriginX(), position.x());
  EXPECT_EQ(costmap2d.getOriginY(), position.y());
  */
}
