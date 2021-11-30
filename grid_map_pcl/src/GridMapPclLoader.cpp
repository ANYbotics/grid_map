/*
 * GridMapPclLoader.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <chrono>

#ifdef GRID_MAP_PCL_OPENMP_FOUND
#include <omp.h>
#endif

#include <pcl/common/io.h>
#include <ros/console.h>

#include <grid_map_core/GridMapMath.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace grid_map {

const grid_map::GridMap& GridMapPclLoader::getGridMap() const {
  return workingGridMap_;
}

void GridMapPclLoader::loadCloudFromPcdFile(const std::string& filename) {
  Pointcloud::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud = grid_map_pcl::loadPointcloudFromPcd(filename);
  setInputCloud(inputCloud);
}

void GridMapPclLoader::setInputCloud(Pointcloud::ConstPtr inputCloud) {
  setRawInputCloud(inputCloud);
  setWorkingCloud(inputCloud);
}

void GridMapPclLoader::setRawInputCloud(Pointcloud::ConstPtr rawInputCloud) {
  rawInputCloud_.reset();
  Pointcloud::Ptr temp(new Pointcloud());
  pcl::copyPointCloud(*rawInputCloud, *temp);
  rawInputCloud_ = temp;
}

void GridMapPclLoader::setWorkingCloud(Pointcloud::ConstPtr workingCloud) {
  workingCloud_.reset();
  Pointcloud::Ptr temp(new Pointcloud());
  pcl::copyPointCloud(*workingCloud, *temp);
  workingCloud_ = temp;
}

void GridMapPclLoader::preProcessInputCloud() {
  // Preprocess: Remove outliers, downsample cloud, transform cloud
  ROS_INFO_STREAM("Preprocessing of the pointcloud started");

  if (params_.get().outlierRemoval_.isRemoveOutliers_) {
    auto filteredCloud = pointcloudProcessor_.removeOutliersFromInputCloud(workingCloud_);
    setWorkingCloud(filteredCloud);
  }

  if (params_.get().downsampling_.isDownsampleCloud_) {
    auto downsampledCloud = pointcloudProcessor_.downsampleInputCloud(workingCloud_);
    setWorkingCloud(downsampledCloud);
  }

  auto transformedCloud = pointcloudProcessor_.applyRigidBodyTransformation(workingCloud_);
  setWorkingCloud(transformedCloud);
  ROS_INFO_STREAM("Preprocessing and filtering finished");
}

void GridMapPclLoader::initializeGridMapGeometryFromInputCloud() {
  workingGridMap_.clearAll();
  const double resolution = params_.get().gridMap_.resolution_;
  if (resolution < 1e-4) {
    throw std::runtime_error("Desired grid map resolution is zero");
  }

  // find point cloud dimensions
  // min and max coordinate in x,y and z direction
  pcl::PointXYZ minBound;
  pcl::PointXYZ maxBound;
  pcl::getMinMax3D(*workingCloud_, minBound, maxBound);

  // from min and max points we can compute the length
  grid_map::Length length = grid_map::Length(maxBound.x - minBound.x, maxBound.y - minBound.y);

  // we put the center of the grid map to be in the middle of the point cloud
  grid_map::Position position = grid_map::Position((maxBound.x + minBound.x) / 2.0, (maxBound.y + minBound.y) / 2.0);
  workingGridMap_.setGeometry(length, resolution, position);

  ROS_INFO_STREAM("Grid map dimensions: " << workingGridMap_.getLength()(0) << " x " << workingGridMap_.getLength()(1));
  ROS_INFO_STREAM("Grid map resolution: " << workingGridMap_.getResolution());
  ROS_INFO_STREAM("Grid map num cells: " << workingGridMap_.getSize()(0) << " x " << workingGridMap_.getSize()(1));
  ROS_INFO_STREAM("Initialized map geometry");
}

void GridMapPclLoader::addLayerFromInputCloud(const std::string& layer) {
  ROS_INFO_STREAM("Started adding layer: " << layer);
  // Preprocess: allocate memory in the internal data structure
  preprocessGridMapCells();
  ROS_INFO("Finished preprocessing");
  workingGridMap_.add(layer);
  grid_map::Matrix& gridMapData = workingGridMap_.get(layer);
  unsigned int linearGridMapSize = workingGridMap_.getSize().prod();

  // Iterate through grid map and calculate the corresponding height based on the point cloud
#ifndef GRID_MAP_PCL_OPENMP_FOUND
  ROS_WARN_STREAM("OpemMP not found, defaulting to single threaded implementation");
#else
  omp_set_num_threads(params_.get().numThreads_);
#pragma omp parallel for schedule(dynamic, 10)
#endif
  for (unsigned int linearIndex = 0; linearIndex < linearGridMapSize; ++linearIndex) {
    processGridMapCell(linearIndex, &gridMapData);
  }
  ROS_INFO_STREAM("Finished adding layer: " << layer);
}

void GridMapPclLoader::processGridMapCell(const unsigned int linearGridMapIndex, grid_map::Matrix* gridMapData) {
  // Get grid map index from linear index and check if enough points lie within the cell
  const grid_map::Index index(grid_map::getIndexFromLinearIndex(linearGridMapIndex, workingGridMap_.getSize()));

  Pointcloud::Ptr pointsInsideCellBorder(new Pointcloud());
  pointsInsideCellBorder = getPointcloudInsideGridMapCellBorder(index);
  const bool isTooFewPointsInCell = pointsInsideCellBorder->size() < params_.get().gridMap_.minCloudPointsPerCell_;
  if (isTooFewPointsInCell) {
    ROS_WARN_STREAM_THROTTLE(10.0, "Less than " << params_.get().gridMap_.minCloudPointsPerCell_ << " points in a cell. Skipping.");
    return;
  }
  if (pointsInsideCellBorder->size() > params_.get().gridMap_.maxCloudPointsPerCell_) {
    ROS_WARN_STREAM_THROTTLE(10.0, "More than " << params_.get().gridMap_.maxCloudPointsPerCell_ << " points in a cell. Skipping.");
    return;
  }
  auto& clusterHeights = clusterHeightsWithingGridMapCell_[index(0)][index(1)];
  calculateElevationFromPointsInsideGridMapCell(pointsInsideCellBorder, clusterHeights);
  if (clusterHeights.empty()) {
    (*gridMapData)(index(0), index(1)) = std::nan("1");
  } else {
    (*gridMapData)(index(0), index(1)) = params_.get().clusterExtraction_.useMaxHeightAsCellElevation_
                                             ? *(std::max_element(clusterHeights.begin(), clusterHeights.end()))
                                             : *(std::min_element(clusterHeights.begin(), clusterHeights.end()));
  }
}

void GridMapPclLoader::calculateElevationFromPointsInsideGridMapCell(Pointcloud::ConstPtr cloud, std::vector<float>& heights) const {
  heights.clear();
  // Extract point cloud cluster from point cloud and return if none is found.
  std::vector<Pointcloud::Ptr> clusterClouds = pointcloudProcessor_.extractClusterCloudsFromPointcloud(cloud);
  const bool isNoClustersFound = clusterClouds.empty();
  if (isNoClustersFound) {
    ROS_WARN_STREAM_THROTTLE(10.0, "No clusters found in the grid map cell");
    return;
  }

  // Extract mean z value of cluster vector and return smallest height value
  heights.reserve(clusterClouds.size());

  std::transform(clusterClouds.begin(), clusterClouds.end(), std::back_inserter(heights),
                 [this](Pointcloud::ConstPtr cloud) -> double { return grid_map_pcl::calculateMeanOfPointPositions(cloud).z(); });
}

GridMapPclLoader::Pointcloud::Ptr GridMapPclLoader::getPointcloudInsideGridMapCellBorder(const grid_map::Index& index) const {
  return pointcloudWithinGridMapCell_[index.x()][index.y()];
}

void GridMapPclLoader::loadParameters(const std::string& filename) {
  params_.loadParameters(filename);
  pointcloudProcessor_.loadParameters(filename);
}

void GridMapPclLoader::setParameters(grid_map_pcl::PclLoaderParameters::Parameters parameters) {
  params_.parameters_ = std::move(parameters);
}

void GridMapPclLoader::savePointCloudAsPcdFile(const std::string& filename) const {
  pointcloudProcessor_.savePointCloudAsPcdFile(filename, *workingCloud_);
}

void GridMapPclLoader::preprocessGridMapCells() {
  allocateSpaceForCloudsInsideCells();
  dispatchWorkingCloudToGridMapCells();
}

void GridMapPclLoader::allocateSpaceForCloudsInsideCells() {
  const unsigned int dimX = workingGridMap_.getSize().x() + 1;
  const unsigned int dimY = workingGridMap_.getSize().y() + 1;

  // resize vectors
  pointcloudWithinGridMapCell_.resize(dimX);
  clusterHeightsWithingGridMapCell_.resize(dimX);

  // allocate pointClouds
  for (unsigned int i = 0; i < dimX; ++i) {
    pointcloudWithinGridMapCell_[i].resize(dimY);
    clusterHeightsWithingGridMapCell_[i].resize(dimY);
    for (unsigned int j = 0; j < dimY; ++j) {
      pointcloudWithinGridMapCell_[i][j].reset(new Pointcloud());
      clusterHeightsWithingGridMapCell_[i][j].clear();
    }
  }
}

void GridMapPclLoader::dispatchWorkingCloudToGridMapCells() {
  // For each point in input point cloud, find which grid map
  // cell does it belong to. Then copy that point in the
  // right cell in the matrix of point clouds data structure.
  // This allows for faster access in the clustering stage.
  for (unsigned int i = 0; i < workingCloud_->points.size(); ++i) {
    const Point& point = workingCloud_->points[i];
    const double x = point.x;
    const double y = point.y;
    grid_map::Index index;
    workingGridMap_.getIndex(grid_map::Position(x, y), index);
    pointcloudWithinGridMapCell_[index.x()][index.y()]->push_back(point);
  }
}
}  // namespace grid_map
