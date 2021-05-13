/*
 * SignedDistanceField.hpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_SDF__SIGNEDDISTANCEFIELD_HPP_
#define GRID_MAP_SDF__SIGNEDDISTANCEFIELD_HPP_

#pragma once

#include <grid_map_core/GridMap.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace grid_map
{

class SignedDistanceField
{
public:
  SignedDistanceField();
  virtual ~SignedDistanceField();

  void calculateSignedDistanceField(
    const GridMap & gridMap, const std::string & layer,
    const double heightClearance);
  double getDistanceAt(const Position3 & position) const;
  Vector3 getDistanceGradientAt(const Position3 & position) const;
  double getInterpolatedDistanceAt(const Position3 & position) const;
  void convertToPointCloud(pcl::PointCloud<pcl::PointXYZI> & points) const;

private:
  Matrix getPlanarSignedDistanceField(
    Eigen::Matrix<bool, Eigen::Dynamic,
    Eigen::Dynamic> & data) const;

  Size size_;
  Position position_;
  std::vector<Matrix> data_;
  float maxDistance_;
  float zIndexStartHeight_;
  double resolution_;
  const float lowestHeight_;
};

}  // namespace grid_map

#endif  // GRID_MAP_SDF__SIGNEDDISTANCEFIELD_HPP_
