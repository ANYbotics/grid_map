/*
 * SignedDistanceField.hpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <grid_map_sdf/SignedDistanceField.hpp>
#include <grid_map_core/GridMap.hpp>

// signed distance field
#include "grid_map_sdf/signed_distance_field/dt.h"

using namespace Eigen;

namespace grid_map {

SignedDistanceField::SignedDistanceField()
    : maxDistance_(100),
      zIndexStartHeight_(0.0),
      resolution_(0.0)
{
}

SignedDistanceField::~SignedDistanceField()
{
}

void SignedDistanceField::calculateSignedDistanceField(const GridMap& gridMap, const std::string& layer,
                                                       const double heightClearance)
{
  data_.clear();
  resolution_ = gridMap.getResolution();
  position_ = gridMap.getPosition();
  size_ = gridMap.getSize();
  Matrix map = gridMap.get(layer);

  // Get min and max height and fill the max height in the NAN area.
  const float minHeight = map.minCoeffOfFinites();
  float maxHeight = map.maxCoeffOfFinites();

  for (size_t i = 0; i < map.size(); ++i) {
    if (std::isnan(map(i))) map(i) = maxHeight;
  }

  // Height range of the signed distance field is higher than the max height.
  maxHeight += heightClearance;

  Matrix sdfElevationAbove = Matrix::Ones(map.rows(), map.cols()) * maxDistance_;
  Matrix sdfElevationBelow = Matrix::Ones(map.rows(), map.cols()) * maxDistance_;
  Matrix sdfLayer(map.rows(), map.cols());

  // Calculate signed distance field in downwards direction.
  std::vector<MatrixXf> sdfElevationBelowList;
  for (float h = maxHeight; h > minHeight; h -= resolution_) {
    for (size_t i = 0; i < sdfElevationBelow.size(); ++i) {
      if (sdfElevationBelow(i) == maxDistance_ && map(i) > h) sdfElevationBelow(i) = h - map(i);
      else if (sdfElevationBelow(i) != maxDistance_) sdfElevationBelow(i) -= resolution_;
    }
    sdfElevationBelowList.push_back(sdfElevationBelow);
  }
  zIndexStartHeight_ = minHeight;

  // Calculate signed distance field from bottom.
  for (float h = minHeight; h < maxHeight; h += resolution_) {
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacleFreeField = map.array() < h;
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacleField = obstacleFreeField.array() < 1;
    MatrixXf sdfObstacle = getPlanarSignedDistanceField(obstacleField);
    MatrixXf sdfObstacleFree = getPlanarSignedDistanceField(obstacleFreeField);
    MatrixXf sdf2d;
    // If 2d sdfObstacleFree calculation failed, neglect this sdf
    // to avoid extreme small distances(-INF).
    if ((sdfObstacleFree.array() > 100000).any()) sdf2d = sdfObstacle;
    else sdf2d = sdfObstacle - sdfObstacleFree;
    sdf2d *= resolution_;
    MatrixXf sdfElevationBelow = sdfElevationBelowList.back();
    sdfElevationBelowList.pop_back();
    for (size_t i = 0; i < sdfElevationAbove.size(); ++i) {
      // Update sdfElevation from the layer below.
      if (sdfElevationAbove(i) == maxDistance_ && map(i) < h) sdfElevationAbove(i) = h - map(i);
      else if (sdfElevationAbove(i) != maxDistance_ && map(i) < h) sdfElevationAbove(i) = sdfLayer(i) + resolution_;
      // SDF is the minimum distance between 2d, elevationAbove and elevationBelow.
      sdfLayer(i) = std::min({sdf2d(i), sdfElevationAbove(i), sdfElevationBelow(i)});
    }
    data_.push_back(sdfLayer);
  }
}

Matrix SignedDistanceField::getPlanarSignedDistanceField(Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& data)
{
  image<uchar> *input = new image<uchar>(data.cols(), data.rows(), true);

  for (int y = 0; y < input->height(); y++) {
    for (int x = 0; x < input->width(); x++) {
      imRef(input, x, y) = data(x, y);
    }
  }

  // Compute dt.
  image<float> *out = dt(input);

  Eigen::MatrixXf result(data.rows(), data.cols());

  // Take square roots.
  for (int y = 0; y < out->height(); y++) {
    for (int x = 0; x < out->width(); x++) {
      result(x, y) = sqrt(imRef(out, x, y));
    }
  }
  return result;
}

double SignedDistanceField::getDistanceAt(const Position3& position)
{
  double xCenter = size_.x() / 2;
  double yCenter = size_.y() / 2;
  int i = std::round(size_.x() - xCenter - (position.x() - position_.x()) / resolution_);
  int j = std::round(size_.y() - yCenter - (position.y() - position_.y()) / resolution_);
  int k = std::round((position.z() - zIndexStartHeight_) / resolution_);
  i = std::max(i, 0);
  i = std::min(i, size_.x() - 1);
  j = std::max(j, 0);
  j = std::min(j, size_.y() - 1);
  k = std::max(k, 0);
  k = std::min(k, (int)data_.size() - 1);
  return data_[k](i, j);
}

Vector3 SignedDistanceField::getDistanceGradientAt(const Position3& position)
{
  double dx = (getDistanceAt(position + Vector3(resolution_, 0, 0)) - getDistanceAt(position - Vector3(resolution_, 0, 0))) / (2 * resolution_);
  double dy = (getDistanceAt(position + Vector3(0, resolution_, 0)) - getDistanceAt(position - Vector3(0, resolution_, 0))) / (2 * resolution_);
  double dz = (getDistanceAt(position + Vector3(0, 0, resolution_)) - getDistanceAt(position - Vector3(0, 0, resolution_))) / (2 * resolution_);
  return Vector3(dx, dy, dz);
}

void SignedDistanceField::convertToPointCloud(pcl::PointCloud<pcl::PointXYZI>& points)
{
  double xCenter = size_.x() / 2;
  double yCenter = size_.y() / 2;
  for (int z = 0; z < data_.size(); z++){
    for (int y = 0; y < size_.y(); y++) {
      for (int x = 0; x < size_.x(); x++) {
        double xp = position_.x() + ((size_.x() - x) - xCenter) * resolution_;
        double yp = position_.y() + ((size_.y() - y) - yCenter) * resolution_;
        double zp = zIndexStartHeight_ + z * resolution_;
        pcl::PointXYZI p;
        p.x = xp;
        p.y = yp;
        p.z = zp;
        p.intensity = data_[z](x, y);
        points.push_back(p);
      }
    }
  }
  return;
}

} /* namespace */
