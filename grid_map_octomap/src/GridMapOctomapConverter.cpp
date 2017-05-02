/*
 * GridMapOctomapConverter.cpp
 *
 *  Created on: May 1, 2017
 *      Author: Jeff Delmerico
 *	 Institute: University of Zürich, Robotics and Perception Group
 */

#include "grid_map_octomap/GridMapOctomapConverter.hpp"

namespace grid_map {

GridMapOctomapConverter::GridMapOctomapConverter()
{
}

GridMapOctomapConverter::~GridMapOctomapConverter()
{
}

bool GridMapOctomapConverter::fromOctomap(const octomap::OcTree& octomap,
                                          grid_map::GridMap& gridMap)
{
  if(octomap.getTreeType() != "OcTree")
  {
    std::cout << "Octomap conversion only implemented for standard OcTree type." << std::endl;
    return false;
  }

  // Set geometry TODO figure out whether to center map
  double resolution = octomap.getResolution();
  grid_map::Position3 minBound;
  grid_map::Position3 maxBound;
  octomap.getMetricMin(minBound(0), minBound(1), minBound(2));
  octomap.getMetricMax(maxBound(0), maxBound(1), maxBound(2));
  grid_map::Length length = grid_map::Length(maxBound(0) - minBound(0), maxBound(1) - minBound(1));
  grid_map::Position position = grid_map::Position((maxBound(0) + minBound(0)) / 2.0,
                                                   (maxBound(1) + minBound(1)) / 2.0);
  gridMap.setGeometry(length, resolution, position);
  std::cout << "grid map geometry: " << std::endl;
  std::cout << "Length: [" << length(0) << ", " << length(1) << "]" << std::endl;
  std::cout << "Position: [" << position(0) << ", " << position(1) << "]" << std::endl;
  std::cout << "Resolution: " << resolution << std::endl;

  // leaf iterator

  return true;
}

} /* namespace */
