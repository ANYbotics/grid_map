/*
 * GridMapPclLoader.hpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <array>
#include <memory>
#include <string>

#include <grid_map_core/GridMap.hpp>

#include "grid_map_pcl/PclLoaderParameters.hpp"
#include "grid_map_pcl/PointcloudProcessor.hpp"

namespace grid_map {

namespace grid_map_pcl_test {
class GridMapPclLoaderTest_CalculateElevation_Test;
}
/*
 * Computes elevation of a grid map from a raw point cloud.
 * The algorithm divides the point cloud into cells (in x and y direction)
 * that are the same size as the grid map cells. Then it looks for clusters
 * of points in each of those cells and computes their mean. Mean of a cluster
 * with minimal z value out of all clusters inside a cell is taken to
 * be the elevation in the grid map. This allows the algorithm to also work
 * indoors. All the calculations are performed in the point cloud frame.
 */

class GridMapPclLoader {
  friend class grid_map_pcl_test::GridMapPclLoaderTest_CalculateElevation_Test;

 public:
  using Point = ::pcl::PointXYZ;
  using Pointcloud = ::pcl::PointCloud<Point>;

  GridMapPclLoader() = default;
  ~GridMapPclLoader() = default;

  /*!
   * Loads the point cloud into memory
   * @param[in] fullpath to the point cloud.
   */
  void loadCloudFromPcdFile(const std::string& filename);

  /*!
   * Allows the user to set the input cloud
   * @param[in] pointer to the input point cloud.
   */
  void setInputCloud(Pointcloud::ConstPtr inputCloud);

  /*!
   * Preprocessing of the input cloud. It removes the outliers,
   * downsamples the cloud and applies a rigid body transform.
   * Parameters are specified in the config file.
   */
  void preProcessInputCloud();

  /*!
   * Initializes the geometry of a grid map from an input cloud. This will
   * set the center of the map and the dimensions in x and y direction. This
   * method will clear any layers that the working grid map might have had.
   */
  void initializeGridMapGeometryFromInputCloud();

  /*!
   * Adds a layer in the grid map. The algorithm is described above.
   * @param[in] Layer name that will be added
   */
  void addLayerFromInputCloud(const std::string& layer);

  /*!
   * Get a const reference to a grid map
   * @param[out] grid map
   */
  const grid_map::GridMap& getGridMap() const;

  /*!
   * Saves a point cloud to a pcd file.
   * @param[in] full path to the output cloud
   */
  void savePointCloudAsPcdFile(const std::string& filename) const;

  /*!
   * Load algorithm's parameters.
   * @param[in] full path to the config file with parameters
   */
  void loadParameters(const std::string& filename);

  /*!
   * Set algorithm's parameters.
   * @param[in] parameters of the algorithm
   */
  void setParameters(const grid_map_pcl::PclLoaderParameters::Parameters parameters);

  //! @return the parameters.
  const grid_map_pcl::PclLoaderParameters::Parameters& getParameters() const { return params_.get(); }

  //! @return A reference to the internal gridMap.
  grid_map::GridMap& getWorkingGridMap() { return workingGridMap_; }

  //! @return A const reference to the internal multiple heights representation.
  const std::vector<std::vector<std::vector<float>>>& getClusterHeightsWithingGridMapCell() const {
    return clusterHeightsWithingGridMapCell_;
  }

 protected:
  /*!
   * Copies the input cloud into the memory. This cloud is expected to be
   * changed if you run the pcl filters. For example if you run the
   * statistical outlier removal, it will remove some points from
   * the cloud
   * @param[in] pointer to the pcl point cloud
   */
  void setWorkingCloud(Pointcloud::ConstPtr workingCloud);

  /*!
   * Copies the input cloud in to the memory. This cloud will not
   * change. Should you need to do any other operations where you want to have
   * the access to the raw point cloud (before applying filters) you can use
   * use the corresponding variable in the class.
   * @param[in] pointer to the pcl point cloud
   */
  void setRawInputCloud(Pointcloud::ConstPtr rawInputCloud);

  // processing the grid map

  /*!
   * @param[in] index of a cell in the grid map
   * @return Point cloud made from points in the working point cloud that fall within
   * the requested cell in the grid map.
   */
  Pointcloud::Ptr getPointcloudInsideGridMapCellBorder(const grid_map::Index& index) const;

  /*!
   * Calculates the elevation at the linear index. The
   * algorithm is applied for each cell after the pre-processing has been done.
   * This function is designed such that it can be called in a parallel for loop.
   * There are no race conditions since each thread processed different
   * set of grid map cells.
   * @param[in] linear index of a grid map cell currently being processed
   * @param[out] matrix of elevation values which need to be computed
   */
  void processGridMapCell(const unsigned int linearGridMapIndex, grid_map::Matrix* gridMapData);

  /*!
   * Given a point cloud it computes the elevation from it. The algorithm is suited for 2.5 D
   * maps. Function finds all the clusters and for each of them it computes a mean of point
   * positions. Then it looks at z value of the means and takes the lowest one.
   * @param[in] point cloud that is entirely contained inside a grid map cell
   * @return elevation value computed from the input point cloud. It will return NaN if no clusters
   * have been found or an empty cloud is passed in.
   */
  void calculateElevationFromPointsInsideGridMapCell(Pointcloud::ConstPtr cloud, std::vector<float>& heights) const;

  /*!
   * Allocates space for the point clouds and dispatches points to the
   * right location in the point cloud matrix. The function merely calls
   * allocateSpaceForCloudsInsideCells and dispatchWorkingCloudToGridMapCells.
   */
  void preprocessGridMapCells();

  /*!
   * Allocates space for the point clouds.  These point clouds are then filled by
   * dispatchWorkingCloudToGridMapCells function.
   */
  void allocateSpaceForCloudsInsideCells();

  /*!
   * Makes a matrix of empty point clouds where each cell in the matrix
   * corresponds to a cell in the grid map. The functions iterates over the working
   * point cloud, checks for each point which cell in the grid map it falls
   * within and then adds that point to the correct point cloud in the matrix of point
   * clouds. The decision which cell in the matrix point cloud a point belongs to
   * or not is made by looking at the xy coordinates of the point in the input point cloud
   * and x-y borders of the cell in the grid map.
   */
  void dispatchWorkingCloudToGridMapCells();

  // Matrix of point clouds. Each point cloud has only points that fall within a grid map cell.
  std::vector<std::vector<Pointcloud::Ptr>> pointcloudWithinGridMapCell_;

  // Matrix of cluster-height-vectors. Dimensions: x, y, cluster_index.
  std::vector<std::vector<std::vector<float>>> clusterHeightsWithingGridMapCell_;

  // Point cloud that pcl filters have been applied to (it can change).
  Pointcloud::Ptr workingCloud_;

  // Copy of the raw point cloud. This cloud will not be changed by pcl filters.
  Pointcloud::Ptr rawInputCloud_;

  // Grid map computed from working point cloud.
  grid_map::GridMap workingGridMap_;

  // Parameters for the algorithm. Also includes parameters for the pcl filters.
  grid_map_pcl::PclLoaderParameters params_;

  // Class that handles point cloud processing
  grid_map_pcl::PointcloudProcessor pointcloudProcessor_;
};

}  // namespace grid_map
