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
                                          grid_map::GridMap& gridmap,
                                          const grid_map::Position3* min_point,
                                          const grid_map::Position3* max_point)
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
  unsigned int max_depth = octomap.getTreeDepth();
  // Adapted from octomap octree2pointcloud.cpp
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (octomap::OcTree::iterator it = octomap.begin(); it != octomap.end(); ++it)
    {
      if(octomap.isNodeOccupied(*it) && it.getDepth() < max_depth)
      {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin();
                                            it != collapsed_occ_nodes.end(); ++it)
    {
      #if OCTOMAP_VERSION_BEFORE_ROS_KINETIC
        (*it)->expandNode();
      #else
        octomap.expandNode(*it);
      #endif
    }
    // std::cout << "Expanded " << collapsed_occ_nodes.size() << " nodes" << std::endl;
  } while(collapsed_occ_nodes.size() > 0);

  // Set up grid map geometry
  // TODO figure out whether to center map
  double resolution = octomap.getResolution();
  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;
  octomap.getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  octomap.getMetricMax(max_bound(0), max_bound(1), max_bound(2));

  // User can provide coordinate limits to only convert a bounding box
  octomap::point3d min_bbx(min_bound(0), min_bound(1), min_bound(2));
  if(min_point)
  {
    min_bbx = octomap::point3d((*min_point)(0), (*min_point)(1), (*min_point)(2));
    min_bound = grid_map::Position3(min_bbx.x(), min_bbx.y(), min_bbx.z());
  }
  octomap::point3d max_bbx(max_bound(0), max_bound(1), max_bound(2));
  if(max_point)
  {
    max_bbx = octomap::point3d((*max_point)(0), (*max_point)(1), (*max_point)(2));
    max_bound = grid_map::Position3(max_bbx.x(), max_bbx.y(), max_bbx.z());
  }

  grid_map::Length length = grid_map::Length(max_bound(0) - min_bound(0), max_bound(1) - min_bound(1));
  grid_map::Position position = grid_map::Position((max_bound(0) + min_bound(0)) / 2.0,
                                                   (max_bound(1) + min_bound(1)) / 2.0);
  gridmap.setGeometry(length, resolution, position);
  // std::cout << "grid map geometry: " << std::endl;
  // std::cout << "Length: [" << length(0) << ", " << length(1) << "]" << std::endl;
  // std::cout << "Position: [" << position(0) << ", " << position(1) << "]" << std::endl;
  // std::cout << "Resolution: " << resolution << std::endl;

  // Add elevation layer
  gridmap.add("elevation");
  gridmap.setBasicLayers({"elevation"});

  // For each voxel, if its elevation is higher than the existing value for the
  // corresponding grid map cell, overwrite it.
  // std::cout << "Iterating from " << min_bbx << " to " << max_bbx << std::endl;
  for(octomap::OcTree::leaf_bbx_iterator it = octomap.begin_leafs_bbx(min_bbx, max_bbx),
          end = octomap.end_leafs_bbx(); it != end; ++it)
  {
    if(octomap.isNodeOccupied(*it))
    {
      octomap::point3d octo_pos = it.getCoordinate();
      grid_map::Position pos(octo_pos.x(), octo_pos.y());
      grid_map::Index idx;
      gridmap.getIndex(pos, idx);
      // If no elevation has been set, use current elevation
      if(!gridmap.isValid(idx))
      {
        gridmap.at("elevation", idx) = octo_pos.z();
      }
      // Check existing elevation, keep higher
      else
      {
        if(gridmap.at("elevation", idx) < octo_pos.z())
        {
          gridmap.at("elevation", idx) = octo_pos.z();
        }
      }
    }
  }

  return true;
}

} /* namespace */
