/*
 * Polygon.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// STD
#include <vector>

// Eigen
#include <Eigen/Core>

namespace grid_map_lib {

class Polygon
{
 public:

  /*!
   * Default constructor.
   */
  Polygon();

  /*!
   * Constructor with vertices.
   * @param vertices the points of the polygon.
   */
  Polygon(std::vector<Eigen::Vector2d> vertices);

  /*!
   * Destructor.
   */
  virtual ~Polygon();

  /*!
   * Check if point is inside polygon.
   * @param point the point to be checked.
   * @return true if inside, false otherwise.
   */
  bool isInside(const Eigen::Vector2d& point);

  /*!
   * Add a vertex to the polygon
   * @param vertex the point to be added.
   */
  void addVertex(const Eigen::Vector2d& vertex);

  /*!
   * Returns the vertices of the polygon.
   * @return the vertices of the polygon.
   */
  const std::vector<Eigen::Vector2d>& getVertices() const;

 protected:

  //! Vertices of the polygon.
  std::vector<Eigen::Vector2d> vertices_;
};

} /* namespace grid_map_lib */
