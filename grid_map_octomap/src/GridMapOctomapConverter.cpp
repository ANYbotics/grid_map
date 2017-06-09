/*
 * GridMapOctomapConverter.cpp
 *
 *  Created on: May 1, 2017
 *      Author: Jeff Delmerico, Peter Fankhauser
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

bool GridMapOctomapConverter::fromOctomap(const octomap::OcTree& octomap,
                                          const std::string& layer,
                                          grid_map::GridMap& gridMap,
                                          const grid_map::Position3* minPoint,
                                          const grid_map::Position3* maxPoint)
{
  if (octomap.getTreeType() != "OcTree") {
    std::cerr << "Octomap conversion only implemented for standard OcTree type." << std::endl;
    return false;
  }

  // Copy octomap in order to expand any pruned occupied cells and maintain constness of input.
  octomap::OcTree octomapCopy(octomap);

  // Iterate through leaf nodes and project occupied cells to elevation map.
  // On the first pass, expand all occupied cells that are not at maximum depth.
  unsigned int max_depth = octomapCopy.getTreeDepth();
  // Adapted from octomap octree2pointcloud.cpp.
  std::vector<octomap::OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (octomap::OcTree::iterator it = octomapCopy.begin(); it != octomapCopy.end(); ++it) {
      if (octomapCopy.isNodeOccupied(*it) && it.getDepth() < max_depth) {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (std::vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin();
                                            it != collapsed_occ_nodes.end(); ++it) {
      #if OCTOMAP_VERSION_BEFORE_ROS_KINETIC
        (*it)->expandNode();
      #else
        octomapCopy.expandNode(*it);
      #endif
    }
    // std::cout << "Expanded " << collapsed_occ_nodes.size() << " nodes" << std::endl;
  } while (collapsed_occ_nodes.size() > 0);

  // Set up grid map geometry.
  // TODO Figure out whether to center map.
  double resolution = octomapCopy.getResolution();
  grid_map::Position3 minBound;
  grid_map::Position3 maxBound;
  octomapCopy.getMetricMin(minBound(0), minBound(1), minBound(2));
  octomapCopy.getMetricMax(maxBound(0), maxBound(1), maxBound(2));

  // User can provide coordinate limits to only convert a bounding box.
  octomap::point3d minBbx(minBound(0), minBound(1), minBound(2));
  if (minPoint) {
    minBbx = octomap::point3d((*minPoint)(0), (*minPoint)(1), (*minPoint)(2));
    minBound = grid_map::Position3(minBbx.x(), minBbx.y(), minBbx.z());
  }
  octomap::point3d maxBbx(maxBound(0), maxBound(1), maxBound(2));
  if (maxPoint) {
    maxBbx = octomap::point3d((*maxPoint)(0), (*maxPoint)(1), (*maxPoint)(2));
    maxBound = grid_map::Position3(maxBbx.x(), maxBbx.y(), maxBbx.z());
  }

  grid_map::Length length = grid_map::Length(maxBound(0) - minBound(0), maxBound(1) - minBound(1));
  grid_map::Position position = grid_map::Position((maxBound(0) + minBound(0)) / 2.0,
                                                   (maxBound(1) + minBound(1)) / 2.0);
  gridMap.setGeometry(length, resolution, position);
  // std::cout << "grid map geometry: " << std::endl;
  // std::cout << "Length: [" << length(0) << ", " << length(1) << "]" << std::endl;
  // std::cout << "Position: [" << position(0) << ", " << position(1) << "]" << std::endl;
  // std::cout << "Resolution: " << resolution << std::endl;

  // Add elevation layer
  gridMap.add(layer);
  gridMap.setBasicLayers({layer});

  // For each voxel, if its elevation is higher than the existing value for the
  // corresponding grid map cell, overwrite it.
  // std::cout << "Iterating from " << min_bbx << " to " << max_bbx << std::endl;
  grid_map::Matrix& gridMapData = gridMap[layer];
  for(octomap::OcTree::leaf_bbx_iterator it = octomapCopy.begin_leafs_bbx(minBbx, maxBbx),
          end = octomapCopy.end_leafs_bbx(); it != end; ++it) {
    if (octomapCopy.isNodeOccupied(*it)) {
      octomap::point3d octoPos = it.getCoordinate();
      grid_map::Position position(octoPos.x(), octoPos.y());
      grid_map::Index index;
      gridMap.getIndex(position, index);
      // If no elevation has been set, use current elevation.
      if (!gridMap.isValid(index)) {
        gridMapData(index(0), index(1)) = octoPos.z();
      }
      // Check existing elevation, keep higher.
      else {
        if (gridMapData(index(0), index(1)) < octoPos.z()) {
          gridMapData(index(0), index(1)) = octoPos.z();
        }
      }
    }
  }

  return true;
}

} /* namespace */
