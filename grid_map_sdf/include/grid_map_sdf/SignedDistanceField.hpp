/*
 * SignedDistanceField.hpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <grid_map_core/GridMap.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <string>
#include <vector>

using namespace grid_map;

namespace grid_map {

class SignedDistanceField
{
 public:
  SignedDistanceField();
  virtual ~SignedDistanceField();

  void calculateSignedDistanceField(const GridMap& gridMap, const std::string& layer, const double heightClearance);
  double getDistanceAt(const Position3& position);
  Vector3 getDistanceGradientAt(const Position3& position);
  void convertToPointCloud(pcl::PointCloud<pcl::PointXYZI>& points);

 private:
  Matrix getPlanarSignedDistanceField(Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& data);

  double resolution_;
  Size size_;
  Position position_;
  std::vector<Matrix> data_;
  double zIndexStartHeight_;
  double maxDistance_;
};

} /* namespace */
