/*
 * GridMapPclConverter.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: Dominic Jud
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/GridMapPclConverter.hpp"

namespace grid_map {

GridMapPclConverter::GridMapPclConverter()
{
}

GridMapPclConverter::~GridMapPclConverter()
{
}

bool GridMapPclConverter::initializeFromPolygonMesh(const pcl::PolygonMesh& mesh,
                                                    const double resolution,
                                                    grid_map::GridMap& gridMap)
{
  pcl::PointCloud < pcl::PointXYZ > cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  pcl::PointXYZ minBound;
  pcl::PointXYZ maxBound;
  pcl::getMinMax3D(cloud, minBound, maxBound);

  grid_map::Length length = grid_map::Length(maxBound.x - minBound.x, maxBound.y - minBound.y);
  grid_map::Position position = grid_map::Position((maxBound.x + minBound.x) / 2.0,
                                                   (maxBound.y + minBound.y) / 2.0);
  gridMap.setGeometry(length, resolution, position);

  return true;
}

bool GridMapPclConverter::addLayerFromPolygonMesh(const pcl::PolygonMesh& mesh,
                                                  const std::string& layer,
                                                  grid_map::GridMap& gridMap)
{

  // Adding a layer to the grid map to put data into
  gridMap.add(layer);

  // Converting out of binary cloud data
  pcl::PointCloud <pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);

  // Stuff for projection
  const Eigen::Vector3f ray = -Eigen::Vector3f::UnitZ();
  pcl::PointXYZ minBound;
  pcl::PointXYZ maxBound;
  pcl::getMinMax3D(cloud, minBound, maxBound);

  // DEBUG
  size_t num_polygons = mesh.polygons.size();

  // Iterating over the triangles
  size_t polygon_counter = 0;
  for (const pcl::Vertices& polygon : mesh.polygons) {

    // DEBUG
    std::cout << "polygon: " << polygon_counter << "/" << num_polygons << std::endl;

    // Testing this is a triangle
    assert(polygon.vertices.size() == 3);

    // Getting the vertices of the triangle (as a single matrix)
    Eigen::Matrix3f triangleVertexMatrix;
    triangleVertexMatrix.row(0) = cloud[polygon.vertices[0]].getVector3fMap();
    triangleVertexMatrix.row(1) = cloud[polygon.vertices[1]].getVector3fMap();
    triangleVertexMatrix.row(2) = cloud[polygon.vertices[2]].getVector3fMap();







    // Constructing the polygon out of the down projection
    std::vector<grid_map::Position> verticesXY;
    for (size_t vertex_idx = 0; vertex_idx < 3; vertex_idx++ ) {
      grid_map::Position vertexXY(
          static_cast<double>(triangleVertexMatrix(vertex_idx, 0)),
          static_cast<double>(triangleVertexMatrix(vertex_idx, 1)));
      verticesXY.push_back(vertexXY);
    }
    grid_map::Polygon triangleXY(verticesXY);

    // Iterating and ray casting
    for (grid_map::PolygonIterator iterator(gridMap, triangleXY);
         !iterator.isPastEnd(); ++iterator) {
      // Cell position
      const Index index(*iterator);
      grid_map::Position vertexPositionXY;
      gridMap.getPosition(index, vertexPositionXY);
      // Ray casting point
      Eigen::Vector3f point(vertexPositionXY.x(), vertexPositionXY.y(),
                            maxBound.z + 1.0);
      // Vertical ray/triangle intersection
      Eigen::Vector3f intersectionPoint;
      if (rayTriangleIntersect(point, ray, triangleVertexMatrix,
                               intersectionPoint)) {
        // If data already present in this cell, taking the max, else setting
        // the data
        if (gridMap.isValid(index, layer)) {
          gridMap.at(layer, index) =
              std::max(gridMap.at(layer, index), intersectionPoint.z());
        } else {
          gridMap.at(layer, index) = intersectionPoint.z();
        }
      }
    }





/*    // Getting the bounds in the XY plane (down projection)
    float maxX = triangleVertexMatrix.col(0).maxCoeff();
    float minX = triangleVertexMatrix.col(0).minCoeff();
    float maxY = triangleVertexMatrix.col(1).maxCoeff();
    float minY = triangleVertexMatrix.col(1).minCoeff();

    // Extracting the submap geometry
    grid_map::Length length(maxX - minX, maxY - minY);
    grid_map::Position position((maxX + minX) / 2.0, (maxY + minY) / 2.0);
    bool isSuccess;
    SubmapGeometry submap(gridMap, position, length, isSuccess);

    // Iterating over the grid cells in the this submap (below the triangle)
    if (isSuccess) {
      for (grid_map::SubmapIterator iterator(submap); !iterator.isPastEnd();
           ++iterator) {
        // Cell position
        const Index index(*iterator);
        grid_map::Position vertexPositionXY;
        gridMap.getPosition(index, vertexPositionXY);
        // Ray casting point
        Eigen::Vector3f point(vertexPositionXY.x(), vertexPositionXY.y(),
                              maxBound.z + 1.0);
        // Vertical ray/triangle intersection
        Eigen::Vector3f intersectionPoint;
        if (rayTriangleIntersect(point, ray, triangleVertexMatrix, intersectionPoint)) {
          // If data already present in this cell, taking the max, else setting the data
          if (gridMap.isValid(index, layer)) {
            gridMap.at(layer, index) = std::max(gridMap.at(layer, index), intersectionPoint.z());
          } else {
            gridMap.at(layer, index) = intersectionPoint.z();
          }
        }
      }
    }
*/
    // DEBUG
    polygon_counter++;

  }

/*  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    grid_map::Position vertexPositionXY;
    gridMap.getPosition(index, vertexPositionXY);

    pcl::PointXYZ point;
    point.x = vertexPositionXY.x();
    point.y = vertexPositionXY.y();
    point.z = maxBound.z + 1.0;

    std::vector<double> candidatePoints;
    for (unsigned i = 0; i < mesh.polygons.size(); ++i) {
      pcl::PointXYZ intersectionPoint;
      if (rayTriangleIntersect(point, ray, mesh.polygons[i], cloud, intersectionPoint))
        candidatePoints.push_back(intersectionPoint.z);
    }
    if (candidatePoints.size() > 0) {
      gridMap.at(layer, index) =
          *(std::max_element(candidatePoints.begin(), candidatePoints.end()));
    } else {
      gridMap.at(layer, index) = NAN;
    }
  }*/

  return true;
}

/*void GridMapPclConverter::testInside(const pcl::PointCloud<pcl::PointXYZ>& pointCloud,
                                     const std::vector<pcl::Vertices>& polygons,
                                     const grid_map::Position& vertexPositionXY) {

  //
  for (const pcl::Vertices& polygon : polygons) {
    // Testing this is a triangle
    assert(polygon.vertices.size() == 3);

    // Getting the 2D, down projected corners of the triangle
    const Eigen::Vector2f a = pointCloud[polygon.vertices[0]].getVector3fMap().head<2>();
    const Eigen::Vector2f b = pointCloud[polygon.vertices[1]].getVector3fMap().head<2>();
    const Eigen::Vector2f c = pointCloud[polygon.vertices[2]].getVector3fMap().head<2>();

    // Getting the side vectors
    const Eigen::Vector2f sv1 = a - b;
    const Eigen::Vector2f sv2 = b - c;
    const Eigen::Vector2f sv3 = c - a;


  }

}*/


/*bool GridMapPclConverter::rayTriangleIntersect(const pcl::PointXYZ& point,
                                               const Eigen::Vector3f& ray,
                                               const pcl::Vertices& vertices,
                                               const pcl::PointCloud<pcl::PointXYZ>& pointCloud,
                                               pcl::PointXYZ& intersectionPoint)
{
  // Algorithm here is adapted from:
  // http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
  //
  // Original copyright notice:
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.

  assert(vertices.vertices.size() == 3);

  const Eigen::Vector3f p = point.getVector3fMap();
  const Eigen::Vector3f a = pointCloud[vertices.vertices[0]].getVector3fMap();
  const Eigen::Vector3f b = pointCloud[vertices.vertices[1]].getVector3fMap();
  const Eigen::Vector3f c = pointCloud[vertices.vertices[2]].getVector3fMap();
  const Eigen::Vector3f u = b - a;
  const Eigen::Vector3f v = c - a;
  const Eigen::Vector3f n = u.cross(v);
  const float n_dot_ray = n.dot(ray);

  if (std::fabs(n_dot_ray) < 1e-9) return false;

  const float r = n.dot(a - p) / n_dot_ray;

  if (r < 0)
    return false;

  const Eigen::Vector3f w = p + r * ray - a;
  const float denominator = u.dot(v) * u.dot(v) - u.dot(u) * v.dot(v);
  const float s_numerator = u.dot(v) * w.dot(v) - v.dot(v) * w.dot(u);
  const float s = s_numerator / denominator;
  if (s < 0 || s > 1) return false;

  const float t_numerator = u.dot(v) * w.dot(u) - u.dot(u) * w.dot(v);
  const float t = t_numerator / denominator;
  if (t < 0 || s + t > 1) return false;

  Eigen::Vector3f intersecPoint = a + s * u + t * v;

  intersectionPoint.x = intersecPoint.x();
  intersectionPoint.y = intersecPoint.y();
  intersectionPoint.z = intersecPoint.z();

  return true;
}*/

bool GridMapPclConverter::rayTriangleIntersect(
    const Eigen::Vector3f& point, const Eigen::Vector3f& ray,
    const Eigen::Matrix3f& triangleVertexMatrix,
    Eigen::Vector3f& intersectionPoint) {
  // Algorithm here is adapted from:
  // http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
  //
  // Original copyright notice:
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.

  const Eigen::Vector3f p = point;
  const Eigen::Vector3f a = triangleVertexMatrix.row(0);
  const Eigen::Vector3f b = triangleVertexMatrix.row(1);
  const Eigen::Vector3f c = triangleVertexMatrix.row(2);
  const Eigen::Vector3f u = b - a;
  const Eigen::Vector3f v = c - a;
  const Eigen::Vector3f n = u.cross(v);
  const float n_dot_ray = n.dot(ray);

  if (std::fabs(n_dot_ray) < 1e-9) return false;

  const float r = n.dot(a - p) / n_dot_ray;

  if (r < 0) return false;

  const Eigen::Vector3f w = p + r * ray - a;
  const float denominator = u.dot(v) * u.dot(v) - u.dot(u) * v.dot(v);
  const float s_numerator = u.dot(v) * w.dot(v) - v.dot(v) * w.dot(u);
  const float s = s_numerator / denominator;
  if (s < 0 || s > 1) return false;

  const float t_numerator = u.dot(v) * w.dot(u) - u.dot(u) * w.dot(v);
  const float t = t_numerator / denominator;
  if (t < 0 || s + t > 1) return false;

  intersectionPoint = a + s * u + t * v;

  return true;
}

} /* namespace */
