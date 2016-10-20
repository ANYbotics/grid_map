/*
 * GridMapPclConverter.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: Dominic Jud
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/GridMapPclConverter.hpp"

namespace grid_map {

GridMapPclConverter::GridMapPclConverter()
{
}

GridMapPclConverter::~GridMapPclConverter()
{
}

bool GridMapPclConverter::initializeFromPolygonMesh(const pcl::PolygonMesh& mesh, const double resolution,
                                                    grid_map::GridMap& gridMap) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    pcl::PointXYZ minBound;
    pcl::PointXYZ maxBound;
    pcl::getMinMax3D(cloud, minBound, maxBound);

    grid_map::Length length = grid_map::Length(maxBound.x - minBound.x, maxBound.y - minBound.y);
    grid_map::Position position = grid_map::Position((maxBound.x + minBound.x)/2.0, (maxBound.y + minBound.y)/2.0);
    gridMap.setGeometry(length, resolution, position);

    return true;
}

bool GridMapPclConverter::addLayerFromPolygonMesh(const pcl::PolygonMesh& mesh, const std::string& layer,
                                                  grid_map::GridMap& gridMap) {
    const Eigen::Vector3f ray(0.0,0.0,-1.0);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    pcl::PointXYZ minBound;
    pcl::PointXYZ maxBound;
    pcl::getMinMax3D(cloud, minBound, maxBound);

    gridMap.add(layer);
    grid_map::Matrix& data = gridMap[layer];

    for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
        const Index index(*iterator);
        grid_map::Position vertexPositionXY;
        gridMap.getPosition(index, vertexPositionXY);

        pcl::PointXYZ point;
        point.x = vertexPositionXY.x();
        point.y = vertexPositionXY.y();
        point.z = maxBound.z + 1.0;

        std::vector<double> candidatePoints;
        for (unsigned i = 0; i < mesh.polygons.size(); ++i)
        {
            pcl::PointXYZ intersectionPoint;
            if (rayTriangleIntersect(point, ray, mesh.polygons[i], cloud, intersectionPoint))
                candidatePoints.push_back(intersectionPoint.z);
        }
        if (candidatePoints.size() > 0)
        {
            gridMap.at(layer, index) = *(std::max_element(candidatePoints.begin(), candidatePoints.end()));
        }
        else
            gridMap.at(layer, index) = NAN;
    }

    return true;
}

bool GridMapPclConverter::rayTriangleIntersect (const pcl::PointXYZ& point,
                                                const Eigen::Vector3f& ray,
                                                const pcl::Vertices& verts,
                                                const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                pcl::PointXYZ& intersectionPoint) {
    // Algorithm here is adapted from:
    // http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
    //
    // Original copyright notice:
    // Copyright 2001, softSurfer (www.softsurfer.com)
    // This code may be freely used and modified for any purpose
    // providing that this copyright notice is included with it.
    //
    assert (verts.vertices.size () == 3);

    const Eigen::Vector3f p = point.getVector3fMap ();
    const Eigen::Vector3f a = cloud[verts.vertices[0]].getVector3fMap ();
    const Eigen::Vector3f b = cloud[verts.vertices[1]].getVector3fMap ();
    const Eigen::Vector3f c = cloud[verts.vertices[2]].getVector3fMap ();
    const Eigen::Vector3f u = b - a;
    const Eigen::Vector3f v = c - a;
    const Eigen::Vector3f n = u.cross (v);
    const float n_dot_ray = n.dot (ray);

    if (std::fabs (n_dot_ray) < 1e-9)
        return false;

    const float r = n.dot (a - p) / n_dot_ray;

    if (r < 0)
        return false;

    const Eigen::Vector3f w = p + r * ray - a;
    const float denominator = u.dot (v) * u.dot (v) - u.dot (u) * v.dot (v);
    const float s_numerator = u.dot (v) * w.dot (v) - v.dot (v) * w.dot (u);
    const float s = s_numerator / denominator;
    if (s < 0 || s > 1)
        return false;

    const float t_numerator = u.dot (v) * w.dot (u) - u.dot (u) * w.dot (v);
    const float t = t_numerator / denominator;
    if (t < 0 || s+t > 1)
        return false;

    Eigen::Vector3f intersecPoint = a + s*u + t*v;

    intersectionPoint.x = intersecPoint.x();
    intersectionPoint.y = intersecPoint.y();
    intersectionPoint.z = intersecPoint.z();

    return true;
}


} /* namespace */
