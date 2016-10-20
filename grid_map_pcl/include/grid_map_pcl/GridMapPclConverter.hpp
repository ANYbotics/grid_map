/*
 * GridMapPclConverter.hpp
 *
 *  Created on: Apr 14, 2016
 *      Author: Dominic Jud
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>

// STD
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

namespace grid_map {

/*!
 * Conversions between grid maps and PCL types.
 */
class GridMapPclConverter
{
 public:
    /*!
     * Default constructor.
     */
    GridMapPclConverter();

    /*!
     * Destructor.
     */
    virtual ~GridMapPclConverter();

    /*!
     * Initializes the geometry of a grid map from a polygon mesh. This changes
     * the geometry of the map and deletes all contents of the layers!
     * @param[in] mesh the mesh.
     * @param[in] resolution the desired resolution of the grid map [m/cell].
     * @param[out] gridMap the grid map to be initialized.
     * @return true if successful, false otherwise.
     */
    static bool initializeFromPolygonMesh(const pcl::PolygonMesh& mesh, const double resolution,
                                          grid_map::GridMap& gridMap);

    /*!
     * Adds a layer with data from a polygon mesh. The mesh is ray traced from
     * above (negative z-Direction).
     * @param[in] mesh the mesh to be added. It can only consist of triangles!
     * @param[in] layer the layer that is filled with the mesh data.
     * @param[out] gridMap the grid map to be populated.
     * @return true if successful, false otherwise.
     */
    static bool addLayerFromPolygonMesh(const pcl::PolygonMesh& mesh, const std::string& layer,
                                        grid_map::GridMap& gridMap);

 private:
    static bool rayTriangleIntersect(const pcl::PointXYZ& point,
                                     const Eigen::Vector3f& ray,
                                     const pcl::Vertices& verts,
                                     const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                     pcl::PointXYZ& intersectionPoint);

};

} /* namespace */