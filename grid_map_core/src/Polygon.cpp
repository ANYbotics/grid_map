/*
 * Polygon.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_core/Polygon.hpp>

namespace grid_map_core {

Polygon::Polygon() {}

Polygon::Polygon(std::vector<Eigen::Vector2d> vertices)
{
  vertices_ = vertices;
}

Polygon::~Polygon() {}

bool Polygon::isInside(const Eigen::Vector2d& point)
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

void Polygon::addVertex(const Eigen::Vector2d& vertex)
{
  vertices_.push_back(vertex);
}

const Eigen::Vector2d& Polygon::getVertex(const size_t index) const
{
  return vertices_.at(index);
}

const std::vector<Eigen::Vector2d>& Polygon::getVertices() const
{
  return vertices_;
}

const size_t Polygon::nVertices() const
{
  return vertices_.size();
}

} /* namespace grid_map */
