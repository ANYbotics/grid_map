/*
 * PointcloudProcessor.cpp
 *
 *  Created on: Nov 21, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/PointcloudProcessor.hpp"

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
#include <memory>

namespace grid_map
{
namespace grid_map_pcl
{

PointcloudProcessor::PointcloudProcessor(const rclcpp::Logger & node_logger)
: node_logger_(node_logger)
{
  params_ = std::make_unique<grid_map_pcl::PclLoaderParameters>(node_logger_);
}

void PointcloudProcessor::loadParameters(const std::string & filename)
{
  params_->loadParameters(filename);
}

Pointcloud::Ptr PointcloudProcessor::removeOutliersFromInputCloud(
  Pointcloud::ConstPtr inputCloud) const
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(inputCloud);
  sor.setMeanK(params_->get().outlierRemoval_.meanK_);
  sor.setStddevMulThresh(params_->get().outlierRemoval_.stddevThreshold_);
  Pointcloud::Ptr filteredCloud(new Pointcloud());
  sor.filter(*filteredCloud);
  return filteredCloud;
}

std::vector<Pointcloud::Ptr> PointcloudProcessor::extractClusterCloudsFromPointcloud(
  Pointcloud::ConstPtr inputCloud) const
{
  std::vector<pcl::PointIndices> clusterIndices = extractClusterIndicesFromPointcloud(inputCloud);
  std::vector<Pointcloud::Ptr> clusterClouds;
  clusterClouds.reserve(clusterIndices.size());
  for (const auto & indicesSet : clusterIndices) {
    Pointcloud::Ptr clusterCloud = makeCloudFromIndices(indicesSet.indices, inputCloud);
    clusterClouds.push_back(clusterCloud);
  }

  return clusterClouds;
}

// todo (jelavice) maybe use the libpointmatcher for this?? faster?
std::vector<pcl::PointIndices> PointcloudProcessor::extractClusterIndicesFromPointcloud(
  Pointcloud::ConstPtr inputCloud) const
{
  // Create a kd tree to cluster the input point cloud
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(inputCloud);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtraction;
  euclideanClusterExtraction.setClusterTolerance(
    params_->get().clusterExtraction_.clusterTolerance_);
  euclideanClusterExtraction.setMinClusterSize(params_->get().clusterExtraction_.minNumPoints_);
  euclideanClusterExtraction.setMaxClusterSize(params_->get().clusterExtraction_.maxNumPoints_);
  euclideanClusterExtraction.setSearchMethod(tree);
  euclideanClusterExtraction.setInputCloud(inputCloud);
  euclideanClusterExtraction.extract(clusterIndices);

  return clusterIndices;
}

Pointcloud::Ptr PointcloudProcessor::makeCloudFromIndices(
  const std::vector<int> & indices,
  Pointcloud::ConstPtr inputCloud) const
{
  Pointcloud::Ptr cloud(new Pointcloud());

  cloud->points.reserve(indices.size());
  for (auto index : indices) {
    cloud->points.push_back(inputCloud->points[index]);
  }

  cloud->is_dense = true;

  return cloud;
}

Pointcloud::Ptr PointcloudProcessor::downsampleInputCloud(Pointcloud::ConstPtr inputCloud) const
{
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(inputCloud);
  const auto & voxelSize = params_->get().downsampling_.voxelSize_;
  voxelGrid.setLeafSize(voxelSize.x(), voxelSize.y(), voxelSize.z());
  Pointcloud::Ptr downsampledCloud(new Pointcloud());
  voxelGrid.filter(*downsampledCloud);
  return downsampledCloud;
}

void PointcloudProcessor::savePointCloudAsPcdFile(
  const std::string & filename,
  const Pointcloud & cloud) const
{
  pcl::PCDWriter writer;
  pcl::PCLPointCloud2 pointCloud2;
  pcl::toPCLPointCloud2(cloud, pointCloud2);
  writer.write(
    filename, pointCloud2, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),
    false);
}

Pointcloud::Ptr PointcloudProcessor::applyRigidBodyTransformation(
  Pointcloud::ConstPtr inputCloud) const
{
  auto transformedCloud = grid_map_pcl::transformCloud(
    inputCloud,
    grid_map_pcl::getRigidBodyTransform(
      params_->get().cloudTransformation_.translation_,
      params_->get().cloudTransformation_.rpyIntrinsic_,
      node_logger_));
  return transformedCloud;
}

}  // namespace grid_map_pcl

}  // namespace grid_map
