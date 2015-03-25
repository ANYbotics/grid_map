/*
 * Polygon.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_core/TypeDefs.hpp>

// STD
#include <vector>

namespace grid_map {

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
  Polygon(std::vector<Position> vertices);

  /*!
   * Destructor.
   */
  virtual ~Polygon();

  /*!
   * Check if point is inside polygon.
   * @param point the point to be checked.
   * @return true if inside, false otherwise.
   */
  bool isInside(const Position& point);

  /*!
   * Add a vertex to the polygon
   * @param vertex the point to be added.
   */
  void addVertex(const Position& vertex);

  /*!
   * Get the vertex with index.
   * @param index the index of the requested vertex.
   * @return the requested vertex.
   */
  const Position& getVertex(const size_t index) const;

  /*!
   * Get vertex operator overload.
   * @param index the index of the requested vertex.
   * @return the requested vertex.
   */
  const Position& operator [](const size_t index) const;

  /*!
   * Returns the vertices of the polygon.
   * @return the vertices of the polygon.
   */
  const std::vector<Position>& getVertices() const;

  /*!
   * Returns the number of vertices.
   * @return the number of vertices.
   */
  const size_t nVertices() const;

  /*!
   * Set the timestamp of the polygon.
   * @param timestamp the timestamp to set (in  nanoseconds).
   */
  void setTimestamp(const uint64_t timestamp);

  /*!
   * Get the timestamp of the polygon.
   * @return timestamp in nanoseconds.
   */
  uint64_t getTimestamp() const;

  /*!
   * Resets the timestamp of the polygon (to zero).
   */
  void resetTimestamp();

  /*!
   * Set the frame id of the polygon.
   * @param frameId the frame id to set.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frameId of the polygon.
   * @return frameId.
   */
  const std::string& getFrameId() const;

 protected:

  //! Frame id of the polygon.
  std::string frameId_;

  //! Timestamp of the polygon (nanoseconds).
  uint64_t timestamp_;

  //! Vertices of the polygon.
  std::vector<Position> vertices_;
};

} /* namespace grid_map */
