/*
 * Signeddistancefield.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Takahiro Miki
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <vector>
#include <string>

#include <grid_map_core/GridMap.hpp>

//pcl
#include <pcl/point_types.h>
#include <pcl/conversions.h>

using namespace grid_map;

namespace grid_map_sdf {

/*!
 * Deletion filter class deletes layers of a grid map.
 */
// template<typename T>
class SignedDistanceField
{
 public:
  /*!
   * Constructor
   */
  SignedDistanceField();

  /*!
   * Destructor.
   */
  virtual ~SignedDistanceField();

  double getDistanceAt(Eigen::Vector3f position);
  Eigen::Vector3f getDistanceGradientAt(Eigen::Vector3f position);
  void calculateSignedDistanceField(GridMap &map, std::string layer, double heightClearance);
  void convertToPointCloud(pcl::PointCloud<pcl::PointXYZI>& points);

 private:

  double resolution_;
  Size size_;
  Position mapPosition_;
  std::vector<Matrix> data_;
  double zIndexStartHeight_;
  double maxDistance_;
  Eigen::MatrixXf get2dSDF(Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& mat);

};

} /* namespace */
