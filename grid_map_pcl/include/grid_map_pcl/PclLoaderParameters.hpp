/*
 * PclLoaderParameters.hpp
 *
 *  Created on: Nov 7, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace grid_map {

class GridMapPclLoader;

namespace grid_map_pcl {

class PointcloudProcessor;

class PclLoaderParameters {
  struct DownsamplingParameters {
    Eigen::Vector3d voxelSize_{0.05, 0.05, 0.05};
    bool isDownsampleCloud_ = false;
  };

  struct ClusterExtractionParameters {
    double clusterTolerance_ = 0.3;
    unsigned int minNumPoints_ = 2;
    unsigned int maxNumPoints_ = 1000000;
    bool useMaxHeightAsCellElevation_;
  };
  struct OutlierRemovalParameters {
    bool isRemoveOutliers_ = false;
    int meanK_ = 10;
    double stddevThreshold_ = 1.0;
  };
  struct RigidBodyTransformation {
    Eigen::Vector3d translation_{0.0, 0.0, 0.0};
    Eigen::Vector3d rpyIntrinsic_{0.0, 0.0, 0.0};  // intrinsic rotation (opposite from the ROS convention), order X-Y-Z
  };

  struct GridMapParameters {
    double resolution_ = 0.1;
    unsigned int minCloudPointsPerCell_ = 2;
    unsigned int maxCloudPointsPerCell_ = 100000;
  };

  struct Parameters {
    unsigned int numThreads_ = 4;
    RigidBodyTransformation cloudTransformation_;
    OutlierRemovalParameters outlierRemoval_;
    ClusterExtractionParameters clusterExtraction_;
    DownsamplingParameters downsampling_;
    GridMapParameters gridMap_;
  };

  friend class grid_map::GridMapPclLoader;
  friend class grid_map::grid_map_pcl::PointcloudProcessor;

 public:
  PclLoaderParameters() = default;
  ~PclLoaderParameters() = default;

  /*!
   * Load parameters for the GridMapPclLoader class.
   * @param[in] full path to the config file with parameters
   * @return true if operation was okay
   */
  bool loadParameters(const std::string& filename);

  /*!
   * Invoke operator[] on the yaml node. Finds
   * the parameters in the yaml tree.
   */
  void loadParameters(const YAML::Node& yamlNode);

  /*!
   * Saves typing parameters_ in the owner class. The owner of this
   * class can type parameters_.get() instead of parameters_.parameters_
   * which would be weird
   */
  const Parameters& get() const;

 private:
  // Parameters for the GridMapPclLoader class.
  Parameters parameters_;
};

} /* namespace grid_map_pcl*/

} /* namespace grid_map */
