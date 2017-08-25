/*
 * NormalVectorsFilter.cpp
 *
 *  Created on: May 05, 2015
 *      Author: Peter Fankhauser, Martin Wermelinger
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <grid_map_filters/NormalVectorsFilter.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>

using namespace filters;

namespace grid_map {

template<typename T>
NormalVectorsFilter<T>::NormalVectorsFilter()
    : estimationRadius_(0.0)
{
}

template<typename T>
NormalVectorsFilter<T>::~NormalVectorsFilter()
{
}

template<typename T>
bool NormalVectorsFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("radius"), estimationRadius_)) {
    ROS_ERROR("Normal vectors filter did not find parameter `radius`.");
    return false;
  }

  if (estimationRadius_ < 0.0) {
    ROS_ERROR("Normal vectors filter estimation radius must be greater than zero.");
    return false;
  }
  ROS_DEBUG("Normal vectors estimation radius = %f", estimationRadius_);

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

  if (!FilterBase < T > ::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Normal vectors filter did not find parameter `input_layer`.");
    return false;
  }
  ROS_DEBUG("Normal vectors filter input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase < T > ::getParam(std::string("output_layers_prefix"), outputLayersPrefix_)) {
    ROS_ERROR("Normal vectors filter did not find parameter `output_layers_prefix`.");
    return false;
  }
  ROS_DEBUG("Normal vectors filter output_layer = %s.", outputLayersPrefix_.c_str());

  return true;
}

template<typename T>
bool NormalVectorsFilter<T>::update(const T& mapIn, T& mapOut)
{
  std::vector<std::string> normalVectorsLayers;
  normalVectorsLayers.push_back(outputLayersPrefix_ + "x");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "y");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "z");

  mapOut = mapIn;
  for (const auto& layer : normalVectorsLayers) mapOut.add(layer);

  // For each cell in requested area.
  for (GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {
    // Check if this is an empty cell (hole in the map).
    if (!mapOut.isValid(*iterator, inputLayer_)) continue;

    // Requested position (center) of circle in map.
    Position center;
    mapOut.getPosition(*iterator, center);

    // Prepare data computation.
    const int maxNumberOfCells = pow(ceil(2 * estimationRadius_ / mapOut.getResolution()), 2);
    Eigen::MatrixXd points(3, maxNumberOfCells);

    // Gather surrounding data.
    size_t nPoints = 0;
    for (CircleIterator iterator(mapOut, center, estimationRadius_); !iterator.isPastEnd(); ++iterator) {
      if (!mapOut.isValid(*iterator, inputLayer_)) continue;
      Position3 point;
      mapOut.getPosition3(inputLayer_, *iterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }
    points.conservativeResize(3, nPoints); // TODO Eigen version?

    // Compute eigenvectors.
    const Position3 mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
    const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;

    const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
    Vector3 eigenvalues = Vector3::Ones();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
    // Ensure that the matrix is suited for eigenvalues calculation.
    if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
      const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
      eigenvalues = solver.eigenvalues().real();
      eigenvectors = solver.eigenvectors().real();
    } else {
      ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);
      // Use z-axis as default surface normal. // TODO Make dependend on surfaceNormalPositiveAxis_;
      eigenvalues.z() = 0.0;
    }
    // Keep the smallest eigenvector as normal vector.
    int smallestId(0);
    double smallestValue(std::numeric_limits<double>::max());
    for (int j = 0; j < eigenvectors.cols(); j++) {
      if (eigenvalues(j) < smallestValue) {
        smallestId = j;
        smallestValue = eigenvalues(j);
      }
    }
    Vector3 eigenvector = eigenvectors.col(smallestId);
    if (eigenvector.dot(normalVectorPositiveAxis_) < 0.0) eigenvector = -eigenvector;
    mapOut.at(outputLayersPrefix_ + "x", *iterator) = eigenvector.x();
    mapOut.at(outputLayersPrefix_ + "y", *iterator) = eigenvector.y();
    mapOut.at(outputLayersPrefix_ + "z", *iterator) = eigenvector.z();
  }

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::NormalVectorsFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
