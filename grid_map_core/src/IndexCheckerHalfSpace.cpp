/*
 * IndexCheckerHalfSpace.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/IndexCheckerHalfSpace.hpp"

namespace grid_map {

IndexCheckerHalfSpace::IndexCheckerHalfSpace(const GridMap& map, const Vector& normal, double offset):
    IndexChecker(map),
    normal_(normal.normalized()),
    offset_(offset){

  // We only need to check one corner of the cell to see if any of the cell lies in the halfspace.
  // Specifically, the opposite corner from
  if (normal.x() >= 0){
    if (normal.y() >= 0){
      cell_corner_offset_ = (map_.getResolution()/2.0)*Position(-1.0,-1.0);
    }
    else
    {
      cell_corner_offset_ = (map_.getResolution()/2.0)*Position(-1.0,1.0);
    }
  }
  else
  {
    if (normal.y() >= 0){
      cell_corner_offset_ = (map_.getResolution()/2.0)*Position(1.0,-1.0);
    }
    else
    {
      cell_corner_offset_ = (map_.getResolution()/2.0)*Position(1.0,1.0);
    }
  }
}

IndexCheckerHalfSpace::IndexCheckerHalfSpace(const GridMap& map, const Vector& normal, const Position& plane_point):
    IndexChecker(map),
    normal_(normal.normalized()){

  offset_ = normal_.dot(plane_point);

  if (normal.x() >= 0){
    if (normal.y() >= 0){
      cell_corner_offset_ = map_.getResolution()*Position(-1.0,-1.0);
    }
    else
    {
      cell_corner_offset_ = map_.getResolution()*Position(-1.0,1.0);
    }
  }
  else
  {
    if (normal.y() >= 0){
      cell_corner_offset_ = map_.getResolution()*Position(1.0,-1.0);
    }
    else
    {
      cell_corner_offset_ = map_.getResolution()*Position(1.0,1.0);
    }
  }
}

bool IndexCheckerHalfSpace::check(const Index& index) const{
  Position center_of_index;
  if (!map_.getPosition(index, center_of_index)){
    return false;
  }

  return normal_.dot(center_of_index+cell_corner_offset_) < offset_;
}

IndexChecker* IndexCheckerHalfSpace::clone() const{
  return (new IndexCheckerHalfSpace(map_, normal_, offset_));
}

}  // namespace grid_map





