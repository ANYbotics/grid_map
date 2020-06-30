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

TEST(OctomapConversion, convertOctomapToGridMap)
{
  // Generate Octomap (a 1m sphere centered at (1,1,1))
  // Adapted from octomap unit_tests.cpp
  octomap::OcTree octomap(0.05);
  octomap::Pointcloud * measurement = new octomap::Pointcloud();

  octomap::point3d origin(1.0f, 1.0f, 1.0f);
  octomap::point3d point_on_surface(1.0f, 0.0f, 0.0f);

  for (int i = 0; i < 360; i++) {
    for (int j = 0; j < 360; j++) {
      octomap::point3d p = origin + point_on_surface;
      measurement->push_back(p);
      point_on_surface.rotate_IP(0, 0, DEG2RAD(1.));
    }
    point_on_surface.rotate_IP(0, DEG2RAD(1.), 0);
  }
  octomap.insertPointCloud(*measurement, origin);

  // Convert to grid map.
  grid_map::GridMap gridMap;
  bool res = grid_map::GridMapOctomapConverter::fromOctomap(octomap, "elevation", gridMap);
  ASSERT_TRUE(res);

  // Check map info.
  grid_map::Length length = gridMap.getLength();
  grid_map::Vector3 octo_length;
  octomap.getMetricSize(octo_length.x(), octo_length.y(), octo_length.z());
  EXPECT_FLOAT_EQ(octo_length.x(), length.x());
  EXPECT_FLOAT_EQ(octo_length.y(), length.y());
  EXPECT_FLOAT_EQ(octomap.getResolution(), gridMap.getResolution());
  EXPECT_FLOAT_EQ(
    2.0 + 0.5 * octomap.getResolution(),
    gridMap.atPosition("elevation", grid_map::Position(1.0, 1.0)));
}

TEST(OctomapConversion, convertOctomapToGridMapWithBoundingBox)
{
  // Repeat test with bounding box

  // Generate Octomap (a 1m sphere centered at (1,1,1))
  // Adapted from Octomap unit_tests.cpp
  octomap::OcTree octomap(0.05);
  octomap::Pointcloud * measurement = new octomap::Pointcloud();

  octomap::point3d origin(1.0f, 1.0f, 1.0f);
  octomap::point3d point_on_surface(1.0f, 0.0f, 0.0f);

  for (int i = 0; i < 360; i++) {
    for (int j = 0; j < 360; j++) {
      octomap::point3d p = origin + point_on_surface;
      measurement->push_back(p);
      point_on_surface.rotate_IP(0, 0, DEG2RAD(1.));
    }
    point_on_surface.rotate_IP(0, DEG2RAD(1.), 0);
  }
  octomap.insertPointCloud(*measurement, origin);

  // Convert to grid map.
  grid_map::GridMap gridmap;
  grid_map::Position3 minpt(0.5, 0.5, 0.0);
  grid_map::Position3 maxpt(2.0, 2.0, 1.0);
  bool res = grid_map::GridMapOctomapConverter::fromOctomap(
    octomap, "elevation", gridmap, &minpt,
    &maxpt);
  ASSERT_TRUE(res);
  grid_map::Length length = gridmap.getLength();
  EXPECT_FLOAT_EQ(1.5, length.x());
  EXPECT_FLOAT_EQ(1.5, length.y());
  EXPECT_FLOAT_EQ(octomap.getResolution(), gridmap.getResolution());
  EXPECT_FLOAT_EQ(
    0.5 * octomap.getResolution(),
    gridmap.atPosition("elevation", grid_map::Position(1.0, 1.0)));
}
