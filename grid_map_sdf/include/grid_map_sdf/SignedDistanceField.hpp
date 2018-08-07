/*
 * SignedDistanceField.hpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_core/GridMap.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <string>
#include <vector>

namespace grid_map {

class SignedDistanceField
{
 public:
  SignedDistanceField();
  virtual ~SignedDistanceField();

  void calculateSignedDistanceField(const GridMap& gridMap, const std::string& layer, const double heightClearance);
  double getDistanceAt(const Position3& position) const;
  Vector3 getDistanceGradientAt(const Position3& position) const;
  double getInterpolatedDistanceAt(const Position3& position) const;
  void convertToPointCloud(pcl::PointCloud<pcl::PointXYZI>& points) const;

 private:
  Matrix getPlanarSignedDistanceField(Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& data) const;

  double resolution_;
  Size size_;
  Position position_;
  std::vector<Matrix> data_;
  float zIndexStartHeight_;
  float maxDistance_;
  const float lowestHeight_;
};

} /* namespace */
