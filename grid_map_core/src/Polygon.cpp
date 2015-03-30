/*
 * Polygon.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_core/Polygon.hpp>

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

} /* namespace grid_map */
