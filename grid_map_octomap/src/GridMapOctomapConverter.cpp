/*
 * GridMapOctomapConverter.cpp
 *
 *  Created on: May 1, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#include "grid_map_octomap/GridMapOctomapConverter.hpp"

namespace grid_map {

GridMapOctomapConverter::GridMapOctomapConverter()
{
}

GridMapOctomapConverter::~GridMapOctomapConverter()
{
}

bool GridMapOctomapConverter::fromOctomap(const octomap::OcTree& octomap_input,
                                          grid_map::GridMap& gridMap)
{
  if(octomap_input.getTreeType() != "OcTree")
  {
    std::cout << "Octomap conversion only implemented for standard OcTree type." << std::endl;
    return false;
  }

  // Copy octomap in order to expand any pruned occupied cells and maintain constness of input
  octomap::OcTree octomap(octomap_input);

  // Iterate through leaf nodes and project occupied cells to elevation map.
  // On the first pass, expand all occupied cells that are not at maximum depth
  unsigned int maxDepth = octomap.getTreeDepth();
  // Adapted from octomap octree2pointcloud.cpp
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (octomap::OcTree::iterator it = octomap.begin(); it != octomap.end(); ++it)
    {
      if(octomap.isNodeOccupied(*it) && it.getDepth() < maxDepth)
      {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin();
                                            it != collapsed_occ_nodes.end(); ++it)
    {
      (*it)->expandNode();
    }
    std::cout << "Expanded " << collapsed_occ_nodes.size() << " nodes" << std::endl;
  } while(collapsed_occ_nodes.size() > 0);

  // Set up grid map geometry
  // TODO figure out whether to center map
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

  // Add elevation layer
  gridMap.add("elevation");
  gridMap.setBasicLayers({"elevation"});

  // For each voxel, if its elevation is higher than the existing value for the
  // corresponding grid map cell, overwrite it.
  for(octomap::OcTree::leaf_iterator it = octomap.begin_leafs(),
          end = octomap.end_leafs(); it != end; ++it)
  {
    if(octomap.isNodeOccupied(*it))
    {
      octomap::point3d octo_pos = it.getCoordinate();
      grid_map::Position pos(octo_pos.x(), octo_pos.y());
      grid_map::Index idx;
      gridMap.getIndex(pos, idx);
      // If no elevation has been set, use current elevation
      if(!gridMap.isValid(idx))
      {
        gridMap.at("elevation", idx) = octo_pos.z();
      }
      // Check existing elevation, keep higher
      else
      {
        if(gridMap.at("elevation", idx) < octo_pos.z())
        {
          gridMap.at("elevation", idx) = octo_pos.z();
        }
      }
    }
  }

  return true;
}

} /* namespace */
