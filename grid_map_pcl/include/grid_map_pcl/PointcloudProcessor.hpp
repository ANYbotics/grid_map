/*
 * PointcloudProcessor.hpp
 *
 *  Created on: Nov 21, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <string>

#include "grid_map_pcl/PclLoaderParameters.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace grid_map {
namespace grid_map_pcl {

class PointcloudProcessor {
 public:
  PointcloudProcessor();
  virtual ~PointcloudProcessor() = default;

  /*!
   * Load parameters for the point cloud processing.
   * @param[in] full path to the config file with parameters
   */
  void loadParameters(const std::string& filename);

  /*!
   * Remove outliers from the point cloud. Function is based on
   * the StatisticalOutlierRemoval filter from pcl. The explanation on
   * how the algorithm works can be found here:
   * http://pointclouds.org/documentation/tutorials/statistical_outlier.php
   * @param[in] Input point cloud
   * @return Point cloud where outliers have been removed.
   */
  Pointcloud::Ptr removeOutliersFromInputCloud(Pointcloud::ConstPtr inputCloud) const;

  /*!
   * Downsample the point cloud using voxel grid method. Implementation is
   * based on the implementation from pcl. The explanation of the algorithm
   * can be found here:
   * http://pointclouds.org/documentation/tutorials/voxel_grid.php
   * @param[in] Input point cloud
   * @return Downsampled point cloud
   */
  Pointcloud::Ptr downsampleInputCloud(Pointcloud::ConstPtr inputCloud) const;

  /*!
   * Finds clusters in the input cloud and returns vector of sets of indices.
   * Each set of indices corresponds to the points in the input cloud that
   * belong to a cluster. There can be more than one cluster.
   * @param[in] pointer to the pcl point cloud
   * @return vector of sets of indices. Vector will be empty if no clusters are found.
   */
  std::vector<pcl::PointIndices> extractClusterIndicesFromPointcloud(Pointcloud::ConstPtr inputCloud) const;

  /*!
   * Finds clusters in the input cloud and returns vector point clouds.
   * Each pointcloud in the vector is a cluster in the input cloud.
   * There can be more than one cluster.
   * @param[in] pointer to the pcl point cloud
   * @return vector of point clouds. Vector will be empty if no clusters are found.
   */
  std::vector<Pointcloud::Ptr> extractClusterCloudsFromPointcloud(Pointcloud::ConstPtr inputCloud) const;

  /*!
   * Given a vector of indices and an input point cloud, the function
   * creates a new cloud that contains points from the input point cloud
   * indexed by the vector of indices.
   * @param[in] vector of indices
   * @param[in] pointer to the pcl point cloud
   * @return Pointer to the point cloud with points indexed by indices.
   */
  static Pointcloud::Ptr makeCloudFromIndices(const std::vector<int>& indices, Pointcloud::ConstPtr inputCloud);

  /*!
   * Saves a point cloud to a pcd file.
   * @param[in] full path to the output cloud
   */
  static void savePointCloudAsPcdFile(const std::string& filename, const Pointcloud& cloud);

  /*!
   * Applies a rigid body transformation to the input cloud. The
   * transformation is specified in the configuration file
   * @param[in] ptr to the input cloud
   * @return resulting cloud when rigid body transformation has been applied
   */
  Pointcloud::Ptr applyRigidBodyTransformation(Pointcloud::ConstPtr inputCloud) const;

 protected:
  // Parameters for the pcl filters.
  std::unique_ptr<grid_map_pcl::PclLoaderParameters> params_;
};

} /* namespace grid_map_pcl */

} /* namespace grid_map*/
