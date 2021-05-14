/*
 * SignedDistanceField.hpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <limits>
#include <algorithm>
#include <string>
#include <vector>

#include "grid_map_sdf/SignedDistanceField.hpp"
#include "grid_map_sdf/distance_transform/dt.hpp"

#include "grid_map_core/GridMap.hpp"


namespace grid_map
{

using namespace distance_transform;  // NOLINT

SignedDistanceField::SignedDistanceField()
: maxDistance_(std::numeric_limits<float>::max()),
  zIndexStartHeight_(0.0),
  resolution_(0.0),
  lowestHeight_(-1e5)     // We need some precision.
{
}

SignedDistanceField::~SignedDistanceField()
{
}

void SignedDistanceField::calculateSignedDistanceField(
  const GridMap & gridMap, const std::string & layer,
  const double heightClearance)
{
  data_.clear();
  resolution_ = gridMap.getResolution();
  position_ = gridMap.getPosition();
  size_ = gridMap.getSize();
  Matrix map = gridMap.get(layer);  // Copy!

  float minHeight = map.minCoeffOfFinites();
  if (!std::isfinite(minHeight)) {minHeight = lowestHeight_;}
  float maxHeight = map.maxCoeffOfFinites();
  if (!std::isfinite(maxHeight)) {maxHeight = lowestHeight_;}

  // maxHeight, minHeight (TODO Make this an option).
  const float valueForEmptyCells = lowestHeight_;
  for (int i = 0; i < map.size(); ++i) {
    if (std::isnan(map(i))) {map(i) = valueForEmptyCells;}
  }

  // Height range of the signed distance field is higher than the max height.
  maxHeight += heightClearance;

  Matrix sdfElevationAbove = Matrix::Ones(map.rows(), map.cols()) * maxDistance_;
  Matrix sdfLayer = Matrix::Zero(map.rows(), map.cols());
  std::vector<Matrix> sdf;
  zIndexStartHeight_ = minHeight;

  // Calculate signed distance field from bottom.
  for (float h = minHeight; h < maxHeight; h += resolution_) {
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacleFreeField = map.array() < h;
    Eigen::Matrix<bool, Eigen::Dynamic,
      Eigen::Dynamic> obstacleField = obstacleFreeField.array() < 1;
    Matrix sdfObstacle = getPlanarSignedDistanceField(obstacleField);
    Matrix sdfObstacleFree = getPlanarSignedDistanceField(obstacleFreeField);
    Matrix sdf2d;
    // If 2d sdfObstacleFree calculation failed, neglect this SDF
    // to avoid extreme small distances (-INF).
    if ((sdfObstacleFree.array() >= distance_transform::INF).any()) {sdf2d = sdfObstacle;} else {
      sdf2d = sdfObstacle - sdfObstacleFree;
    }
    sdf2d *= resolution_;
    for (int i = 0; i < sdfElevationAbove.size(); ++i) {
      if (sdfElevationAbove(i) == maxDistance_ && map(i) <= h) {
        sdfElevationAbove(i) = h - map(i);
      } else if (sdfElevationAbove(i) != maxDistance_ && map(i) <= h) {
        sdfElevationAbove(i) = sdfLayer(i) + resolution_;
      }
      if (sdf2d(i) == 0) {sdfLayer(i) = h - map(i);} else if (sdf2d(i) < 0) {
        sdfLayer(i) = -std::min(fabs(sdf2d(i)), fabs(map(i) - h));
      } else {sdfLayer(i) = std::min(sdf2d(i), sdfElevationAbove(i));}
    }
    data_.push_back(sdfLayer);
  }
}

grid_map::Matrix SignedDistanceField::getPlanarSignedDistanceField(
  Eigen::Matrix<bool,
  Eigen::Dynamic,
  Eigen::Dynamic> & data) const
{
  image<uchar> * input = new image<uchar>(data.rows(), data.cols(), true);

  for (int y = 0; y < input->height(); y++) {
    for (int x = 0; x < input->width(); x++) {
      imRef(input, x, y) = data(x, y);
    }
  }

  // Compute dt.
  image<float> * out = dt(input);

  Matrix result(data.rows(), data.cols());

  // Take square roots.
  for (int y = 0; y < out->height(); y++) {
    for (int x = 0; x < out->width(); x++) {
      result(x, y) = sqrt(imRef(out, x, y));
    }
  }
  return result;
}

double SignedDistanceField::getDistanceAt(const Position3 & position) const
{
  double xCenter = size_.x() / 2.0;
  double yCenter = size_.y() / 2.0;
  int i = std::round(xCenter - (position.x() - position_.x()) / resolution_);
  int j = std::round(yCenter - (position.y() - position_.y()) / resolution_);
  int k = std::round((position.z() - zIndexStartHeight_) / resolution_);
  i = std::max(i, 0);
  i = std::min(i, size_.x() - 1);
  j = std::max(j, 0);
  j = std::min(j, size_.y() - 1);
  k = std::max(k, 0);
  k = std::min(k, static_cast<int>(data_.size()) - 1);
  return data_[k](i, j);
}

double SignedDistanceField::getInterpolatedDistanceAt(const Position3 & position) const
{
  double xCenter = size_.x() / 2.0;
  double yCenter = size_.y() / 2.0;
  int i = std::round(xCenter - (position.x() - position_.x()) / resolution_);
  int j = std::round(yCenter - (position.y() - position_.y()) / resolution_);
  int k = std::round((position.z() - zIndexStartHeight_) / resolution_);
  i = std::max(i, 0);
  i = std::min(i, size_.x() - 1);
  j = std::max(j, 0);
  j = std::min(j, size_.y() - 1);
  k = std::max(k, 0);
  k = std::min(k, static_cast<int>(data_.size()) - 1);
  Vector3 gradient = getDistanceGradientAt(position);
  double xp = position_.x() + ((size_.x() - i) - xCenter) * resolution_;
  double yp = position_.y() + ((size_.y() - j) - yCenter) * resolution_;
  double zp = zIndexStartHeight_ + k * resolution_;
  Vector3 error = position - Vector3(xp, yp, zp);
  return data_[k](i, j) + gradient.dot(error);
}

Vector3 SignedDistanceField::getDistanceGradientAt(const Position3 & position) const
{
  double xCenter = size_.x() / 2.0;
  double yCenter = size_.y() / 2.0;
  int i = std::round(xCenter - (position.x() - position_.x()) / resolution_);
  int j = std::round(yCenter - (position.y() - position_.y()) / resolution_);
  int k = std::round((position.z() - zIndexStartHeight_) / resolution_);
  i = std::max(i, 1);
  i = std::min(i, size_.x() - 2);
  j = std::max(j, 1);
  j = std::min(j, size_.y() - 2);
  k = std::max(k, 1);
  k = std::min(k, static_cast<int>(data_.size()) - 2);
  double dx = (data_[k](i - 1, j) - data_[k](i + 1, j)) / (2 * resolution_);
  double dy = (data_[k](i, j - 1) - data_[k](i, j + 1)) / (2 * resolution_);
  double dz = (data_[k + 1](i, j) - data_[k - 1](i, j)) / (2 * resolution_);
  return Vector3(dx, dy, dz);
}

void SignedDistanceField::convertToPointCloud(pcl::PointCloud<pcl::PointXYZI> & points) const
{
  double xCenter = size_.x() / 2.0;
  double yCenter = size_.y() / 2.0;
  for (int z = 0; z < static_cast<int>(data_.size()); z++) {
    for (int y = 0; y < static_cast<int>(size_.y()); y++) {
      for (int x = 0; x < static_cast<int>(size_.x()); x++) {
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
}

}  // namespace grid_map
