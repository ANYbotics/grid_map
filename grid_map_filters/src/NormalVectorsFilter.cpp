/*
 * NormalVectorsFilter.cpp
 *
 *  Created on: May 05, 2015
 *      Author: Peter Fankhauser, Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_filters/NormalVectorsFilter.hpp>

#include <pluginlib/class_list_macros.h>
#include <grid_map_core/grid_map_core.hpp>

#include <Eigen/Dense>
#include <stdexcept>

using namespace filters;

namespace grid_map {

template <typename T>
NormalVectorsFilter<T>::NormalVectorsFilter() : method_(Method::Raster), estimationRadius_(0.0) {}

template <typename T>
NormalVectorsFilter<T>::~NormalVectorsFilter() {}

template <typename T>
bool NormalVectorsFilter<T>::configure() {
  if (!FilterBase<T>::getParam(std::string("radius"), estimationRadius_)) {
    ROS_DEBUG("Normal vectors filter did not find parameter `radius`.");
    method_ = Method::Raster;
  } else {
    method_ = Method::Area;
    if (estimationRadius_ < 0.0) {
      ROS_ERROR("Normal vectors filter estimation radius must be greater than zero.");
      return false;
    }
    ROS_DEBUG("Normal vectors estimation radius = %f", estimationRadius_);
  }

  std::string normalVectorPositiveAxis;
  if (!FilterBase<T>::getParam(std::string("normal_vector_positive_axis"), normalVectorPositiveAxis)) {
    ROS_ERROR("Normal vectors filter did not find parameter `normal_vector_positive_axis`.");
    return false;
  }
  if (normalVectorPositiveAxis == "z") {
    normalVectorPositiveAxis_ = Vector3::UnitZ();
  } else if (normalVectorPositiveAxis == "y") {
    normalVectorPositiveAxis_ = Vector3::UnitY();
  } else if (normalVectorPositiveAxis == "x") {
    normalVectorPositiveAxis_ = Vector3::UnitX();
  } else {
    ROS_ERROR("The normal vector positive axis '%s' is not valid.", normalVectorPositiveAxis.c_str());
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Normal vectors filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("Normal vectors filter input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase<T>::getParam(std::string("output_layers_prefix"), outputLayersPrefix_)) {
    ROS_ERROR("Normal vectors filter did not find parameter `output_layers_prefix`.");
    return false;
  }
  ROS_DEBUG("Normal vectors filter output_layer = %s.", outputLayersPrefix_.c_str());

  return true;
}

template <typename T>
bool NormalVectorsFilter<T>::update(const T& mapIn, T& mapOut) {
  std::vector<std::string> normalVectorsLayers;
  normalVectorsLayers.push_back(outputLayersPrefix_ + "x");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "y");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "z");

  mapOut = mapIn;
  for (const auto& layer : normalVectorsLayers) {
    mapOut.add(layer);
  }
  switch (method_) {
    case Method::Area:
      computeWithArea(mapOut, inputLayer_, outputLayersPrefix_);
      break;
    case Method::Raster:
      computeWithRaster(mapOut, inputLayer_, outputLayersPrefix_);
      break;
  }

  return true;
}

template <typename T>
void NormalVectorsFilter<T>::computeWithArea(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix) {
  // For each cell in requested area.
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    // Check if this is an empty cell (hole in the map).
    if (!map.isValid(*iterator, inputLayer)) {
      continue;
    }

    // Requested position (center) of circle in map.
    Position center;
    map.getPosition(*iterator, center);

    // Prepare data computation. Check if area is bigger than cell.
    const double minAllowedEstimationRadius = 0.5 * map.getResolution();
    if (estimationRadius_ <= minAllowedEstimationRadius) {
      ROS_WARN("Estimation radius is smaller than allowed by the map resolution (%d < %d)", estimationRadius_, minAllowedEstimationRadius);
    }

    // Gather surrounding data.
    size_t nPoints = 0;
    Position3 sum = Position3::Zero();
    Eigen::Matrix3d sumSquared = Eigen::Matrix3d::Zero();
    for (CircleIterator circleIterator(map, center, estimationRadius_); !circleIterator.isPastEnd(); ++circleIterator) {
      Position3 point;
      if (!map.getPosition3(inputLayer, *circleIterator, point)) {
        continue;
      }
      nPoints++;
      sum += point;
      sumSquared.noalias() += point * point.transpose();
    }

    Vector3 unitaryNormalVector = Vector3::Zero();
    if (nPoints < 3) {
      ROS_DEBUG("Not enough points to establish normal direction (nPoints = %i)", nPoints);
      unitaryNormalVector = {0, 0, 1};
    } else {
      const Position3 mean = sum / nPoints;
      const Eigen::Matrix3d covarianceMatrix = sumSquared / nPoints - mean * mean.transpose();

      // Compute Eigenvectors.
      // Eigenvalues are ordered small to large
      // Worst case bound for zero eigenvalue from : https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
      solver.computeDirect(covarianceMatrix, Eigen::DecompositionOptions::ComputeEigenvectors);
      if (solver.eigenvalues()(1) > 1e-8) {
        unitaryNormalVector = solver.eigenvectors().col(0);
      } else {  // If second eigenvalue is zero, the normal is not defined
        ROS_DEBUG(
            "Covariance matrix needed for eigen decomposition is degenerated. Expected cause: data is on a straight line (nPoints = %i)",
            nPoints);
        unitaryNormalVector = {0, 0, 1};
      }
    }

    // Check direction of the normal vector and flip the sign towards the user defined direction.
    if (unitaryNormalVector.dot(normalVectorPositiveAxis_) < 0.0) {
      unitaryNormalVector = -unitaryNormalVector;
    }

    map.at(outputLayersPrefix + "x", *iterator) = unitaryNormalVector.x();
    map.at(outputLayersPrefix + "y", *iterator) = unitaryNormalVector.y();
    map.at(outputLayersPrefix + "z", *iterator) = unitaryNormalVector.z();
  }
}

template <typename T>
void NormalVectorsFilter<T>::computeWithRaster(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix) {
  throw std::runtime_error("NormalVectorsFilter::computeWithRaster() is not yet implemented!");
  // TODO: http://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::NormalVectorsFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
