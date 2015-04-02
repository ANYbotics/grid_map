/*
 * Polygon.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_core/Polygon.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace grid_map {

Polygon::Polygon() {}

Polygon::Polygon(std::vector<Position> vertices)
{
  vertices_ = vertices;
}

Polygon::~Polygon() {}

bool Polygon::isInside(const Position& point)
{
  int cross = 0;
  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((vertices_[i].y() > point.y()) != (vertices_[j].y() > point.y()))
           && (point.x() < (vertices_[j].x() - vertices_[i].x()) * (point.y() - vertices_[i].y()) /
            (vertices_[j].y() - vertices_[i].y()) + vertices_[i].x()) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}

void Polygon::addVertex(const Position& vertex)
{
  vertices_.push_back(vertex);
}

const Position& Polygon::getVertex(const size_t index) const
{
  return vertices_.at(index);
}

const Position& Polygon::operator [](const size_t index) const
{
  return getVertex(index);
}

const std::vector<Position>& Polygon::getVertices() const
{
  return vertices_;
}

const size_t Polygon::nVertices() const
{
  return vertices_.size();
}

const std::string& Polygon::getFrameId() const
{
  return frameId_;
}

void Polygon::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

uint64_t Polygon::getTimestamp() const
{
  return timestamp_;
}

void Polygon::setTimestamp(const uint64_t timestamp)
{
  timestamp_ = timestamp;
}

void Polygon::resetTimestamp()
{
  timestamp_ = 0.0;
}

Polygon Polygon::convexHullCircles(const Position center1, const Position center2, const double radius)
{
  Eigen::Vector2d centerToVertex, centerToVertexTemp;
  centerToVertex = center2 - center1;
  centerToVertex.normalize();
  centerToVertex *= radius;
  const int nVertices = 10;

  grid_map::Polygon polygon;
  for (int j=0; j<nVertices; j++) {
    double theta = M_PI_2 + j*M_PI/(nVertices-1);
    Eigen::Rotation2D<double> rot2d(theta);
    centerToVertexTemp = rot2d.toRotationMatrix()*centerToVertex;
    polygon.addVertex(center1+centerToVertexTemp);
  }
  for (int j=0; j<nVertices; j++) {
    double theta = 3*M_PI_2 + j*M_PI/(nVertices-1);
    Eigen::Rotation2D<double> rot2d(theta);
    centerToVertexTemp = rot2d.toRotationMatrix()*centerToVertex;
    polygon.addVertex(center2+centerToVertexTemp);
  }
  return polygon;
}

Polygon Polygon::convexHull(Polygon& polygon1, Polygon& polygon2)
{
  std::vector<Eigen::Vector2d> vertices1 = polygon1.getVertices();
  std::vector<Eigen::Vector2d> vertices2 = polygon2.getVertices();
  std::vector<Eigen::Vector2d> vertices = vertices1;
  for (const auto& vertice : vertices2) {
    vertices.push_back(vertice);
  }
  std::vector<Eigen::Vector2d> hull(vertices.size());

  // Sort points lexicographically
  std::sort (vertices.begin(), vertices.end(), sortVertices);

  int k = 0;
  // Build lower hull
  for (int i = 0; i < vertices.size(); ++i) {
    while (k >= 2 && computeCrossProduct2D(hull.at(k-1) - hull.at(k-2), vertices.at(i) - hull.at(k-2)) <= 0) k--;
    hull.at(k++) = vertices.at(i);
  }

  // Build upper hull
  for (int i = vertices.size()-2, t = k+1; i >= 0; i--) {
    while (k >= t && computeCrossProduct2D(hull.at(k-1) - hull.at(k-2), vertices.at(i) - hull.at(k-2)) <= 0) k--;
    hull.at(k++) = vertices.at(i);
  }
  hull.resize(k-1);

  Polygon polygonOut;
  for (const auto& vertex : hull) {
    polygonOut.addVertex(vertex);
  }
  return polygonOut;
}

bool Polygon::sortVertices(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2)
{
  bool isSmaller = false;

  if (vector1.x() < vector2.x() || (vector1.x() == vector2.x() && vector1.y() < vector2.y())) {
    isSmaller = true;
  }

  return isSmaller;
}

double Polygon::computeCrossProduct2D(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2)
{
  return (vector1.x()*vector2.y()-vector1.y()*vector2.x());
}

} /* namespace grid_map */
