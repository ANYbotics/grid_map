/*
 * NormalVectorsFilter.cpp
 *
 *  Created on: May 05, 2015
 *      Author: Peter Fankhauser, Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>

#include <tbb/parallel_for.h>

#include <grid_map_core/grid_map_core.hpp>

#include <grid_map_filters/NormalVectorsFilter.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
NormalVectorsFilter<T>::NormalVectorsFilter()
: method_(Method::RasterSerial), estimationRadius_(0.0), parallelizationEnabled_(false),
  threadCount_(1), gridMapResolution_(0.02) {}

template<typename T>
NormalVectorsFilter<T>::~NormalVectorsFilter() = default;

template<typename T>
bool NormalVectorsFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  // Read which algorithm is chosen: area or raster.
  std::string algorithm;
  if (!param_reader.get(std::string("algorithm"), algorithm)) {
    RCLCPP_WARN(
      this->logging_interface_->get_logger(),
      "Could not find the parameter: `algorithm`. Setting to default value: 'area'.");
    // Default value.
    algorithm = "area";
  }

  // Read parameters related to area algorithm only if needed, otherwise when using raster method
  // on purpose it throws unwanted errors.
  if (algorithm != "raster") {
    // Read radius, if found, its value will be used for area method.
    // If radius parameter is not found, raster method will be used.
    if (!param_reader.get(std::string("radius"), estimationRadius_)) {
      RCLCPP_WARN(
        this->logging_interface_->get_logger(),
        "Could not find the parameter: `radius`. Switching to raster method.");
      algorithm = "raster";
    }
    RCLCPP_DEBUG(
      this->logging_interface_->get_logger(), "Normal vectors estimation radius = %f",
      estimationRadius_);
    // If radius not positive switch to raster method.
    if (estimationRadius_ <= 0) {
      RCLCPP_WARN(
        this->logging_interface_->get_logger(),
        "Parameter `radius` is not positive. Switching to raster method.");
      algorithm = "raster";
    }
  }

  // Read parallelization_enabled to decide whether parallelization has to be used,
  // if parameter is not found an error is thrown and the false default value will be used.
  if (!param_reader.get(std::string("parallelization_enabled"), parallelizationEnabled_)) {
    RCLCPP_WARN(
      this->logging_interface_->get_logger(),
      "Could not find the parameter:"
      " `parallelization_enabled`. Setting to default value: 'false'.");
    parallelizationEnabled_ = false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Parallelization_enabled = %d",
    parallelizationEnabled_);

  // Set wanted method looking at algorithm and parallelization_enabled parameters.
  // parallelization_enabled is used to select whether to use parallelization or not.
  if (algorithm == "raster") {
    // If parallelizationEnabled_=true, use the parallel method, otherwise serial.
    if (parallelizationEnabled_) {
      method_ = Method::RasterParallel;
      RCLCPP_DEBUG(this->logging_interface_->get_logger(), "Method RasterParallel");
    } else {
      method_ = Method::RasterSerial;
      RCLCPP_DEBUG(this->logging_interface_->get_logger(), "Method RasterSerial");
    }
  } else {
    // If parallelizationEnabled_=true, use the parallel method, otherwise serial.
    if (parallelizationEnabled_) {
      method_ = Method::AreaParallel;
      RCLCPP_DEBUG(this->logging_interface_->get_logger(), "Method AreaParallel");
    } else {
      method_ = Method::AreaSerial;
      RCLCPP_DEBUG(this->logging_interface_->get_logger(), "Method AreaSerial");
    }
    RCLCPP_DEBUG(
      this->logging_interface_->get_logger(), "estimationRadius_ = %f",
      estimationRadius_);
  }

  // Read normal_vector_positive_axis, to define normal vector positive direction.
  std::string normalVectorPositiveAxis;
  if (!param_reader.get(
      std::string("normal_vector_positive_axis"),
      normalVectorPositiveAxis))
  {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Normal vectors filter did not find parameter `normal_vector_positive_axis`.");
    return false;
  }
  if (normalVectorPositiveAxis == "z") {
    normalVectorPositiveAxis_ = Vector3::UnitZ();
  } else if (normalVectorPositiveAxis == "y") {
    normalVectorPositiveAxis_ = Vector3::UnitY();
  } else if (normalVectorPositiveAxis == "x") {
    normalVectorPositiveAxis_ = Vector3::UnitX();
  } else {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "The normal vector positive axis '%s' is not valid.",
      normalVectorPositiveAxis.c_str());
    return false;
  }

  // Read input_layer, to define input grid map layer.
  if (!param_reader.get(std::string("input_layer"), inputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Normal vectors filter did not find parameter `input_layer`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Normal vectors filter input layer is = %s.",
    inputLayer_.c_str());

  // Read output_layers_prefix, to define output grid map layers prefix.
  if (!param_reader.get(std::string("output_layers_prefix"), outputLayersPrefix_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Normal vectors filter did not find parameter `output_layers_prefix`.");
    return false;
  }
  RCLCPP_DEBUG(
    this->logging_interface_->get_logger(), "Normal vectors filter output_layer = %s.",
    outputLayersPrefix_.c_str());

  // If everything has been set up correctly
  return true;
}

template<typename T>
bool NormalVectorsFilter<T>::update(const T & mapIn, T & mapOut)
{
  std::vector<std::string> normalVectorsLayers;
  normalVectorsLayers.push_back(outputLayersPrefix_ + "x");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "y");
  normalVectorsLayers.push_back(outputLayersPrefix_ + "z");

  mapOut = mapIn;
  for (const auto & layer : normalVectorsLayers) {
    mapOut.add(layer);
  }
  switch (method_) {
    case Method::AreaSerial:
      computeWithAreaSerial(mapOut, inputLayer_, outputLayersPrefix_);
      break;
    case Method::RasterSerial:
      computeWithRasterSerial(mapOut, inputLayer_, outputLayersPrefix_);
      break;
    case Method::AreaParallel:
      computeWithAreaParallel(mapOut, inputLayer_, outputLayersPrefix_);
      break;
    case Method::RasterParallel:
      computeWithRasterParallel(mapOut, inputLayer_, outputLayersPrefix_);
      break;
  }

  return true;
}

// SVD Area based methods.
template<typename T>
void NormalVectorsFilter<T>::computeWithAreaSerial(
  GridMap & map, const std::string & inputLayer,
  const std::string & outputLayersPrefix)
{
  rclcpp::Clock clock;
  const double start = clock.now().seconds();

  // For each cell in submap.
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    // Check if this is an empty cell (hole in the map).
    if (map.isValid(*iterator, inputLayer)) {
      const Index index(*iterator);
      areaSingleNormalComputation(map, inputLayer, outputLayersPrefix, index);
    }
  }

  const double end = clock.now().seconds();
  RCLCPP_DEBUG_THROTTLE(
    this->logging_interface_->get_logger(), clock, 2.0, "NORMAL COMPUTATION TIME = %f",
    (end - start));
}

template<typename T>
void NormalVectorsFilter<T>::computeWithAreaParallel(
  GridMap & map, const std::string & inputLayer,
  const std::string & outputLayersPrefix)
{
  rclcpp::Clock clock;
  const double start = clock.now().seconds();
  grid_map::Size gridMapSize = map.getSize();

  // Parallelized iteration through the map.
  // Number of threads is automatically determined.
  tbb::parallel_for(
    0, gridMapSize(0) * gridMapSize(1), [&](int range) {
      // Recover Cell index from range iterator.
      const Index index(range / gridMapSize(1), range % gridMapSize(1));
      if (map.isValid(index, inputLayer)) {
        areaSingleNormalComputation(map, inputLayer, outputLayersPrefix, index);
      }
    });

  const double end = clock.now().seconds();
  RCLCPP_DEBUG_THROTTLE(
    this->logging_interface_->get_logger(), clock, 2.0, "NORMAL COMPUTATION TIME = %f",
    (end - start));
}

template<typename T>
void NormalVectorsFilter<T>::areaSingleNormalComputation(
  GridMap & map, const std::string & inputLayer,
  const std::string & outputLayersPrefix, const grid_map::Index & index)
{
  // Requested position (center) of circle in map.
  Position center;
  map.getPosition(index, center);

  // Prepare data computation. Check if area is bigger than cell.
  const double minAllowedEstimationRadius = 0.5 * map.getResolution();
  if (estimationRadius_ <= minAllowedEstimationRadius) {
    RCLCPP_WARN(
      this->logging_interface_->get_logger(),
      "Estimation radius is smaller than allowed by the map resolution (%f < %f)",
      estimationRadius_, minAllowedEstimationRadius);
  }

  // Gather surrounding data.
  size_t nPoints = 0;
  Position3 sum = Position3::Zero();
  Eigen::Matrix3d sumSquared = Eigen::Matrix3d::Zero();
  for (CircleIterator circleIterator(map, center, estimationRadius_); !circleIterator.isPastEnd();
    ++circleIterator)
  {
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
    RCLCPP_DEBUG(
      this->logging_interface_->get_logger(),
      "Not enough points to establish normal direction (nPoints = %i)",
      static_cast<int>(nPoints));
    unitaryNormalVector = Vector3::UnitZ();
  } else {
    const Position3 mean = sum / nPoints;
    const Eigen::Matrix3d covarianceMatrix = sumSquared / nPoints - mean * mean.transpose();

    // Compute Eigenvectors.
    // Eigenvalues are ordered small to large.
    // Worst case bound for zero eigenvalue from : https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.computeDirect(covarianceMatrix, Eigen::DecompositionOptions::ComputeEigenvectors);
    if (solver.eigenvalues()(1) > 1e-8) {
      unitaryNormalVector = solver.eigenvectors().col(0);
    } else {  // If second eigenvalue is zero, the normal is not defined.
      RCLCPP_DEBUG(
        this->logging_interface_->get_logger(),
        "Covariance matrix needed for eigen decomposition is degenerated.");
      RCLCPP_DEBUG(
        this->logging_interface_->get_logger(),
        "Expected cause: data is on a straight line (nPoints = %i)",
        static_cast<int>(nPoints));
      unitaryNormalVector = Vector3::UnitZ();
    }
  }

  // Check direction of the normal vector and flip the sign towards the user defined direction.
  if (unitaryNormalVector.dot(normalVectorPositiveAxis_) < 0.0) {
    unitaryNormalVector = -unitaryNormalVector;
  }

  map.at(outputLayersPrefix + "x", index) = unitaryNormalVector.x();
  map.at(outputLayersPrefix + "y", index) = unitaryNormalVector.y();
  map.at(outputLayersPrefix + "z", index) = unitaryNormalVector.z();
}
// Raster based methods.
template<typename T>
void NormalVectorsFilter<T>::computeWithRasterSerial(
  GridMap & map, const std::string & inputLayer,
  const std::string & outputLayersPrefix)
{
  // Inspiration for algorithm: http://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
  rclcpp::Clock clock;
  const double start = clock.now().seconds();

  const grid_map::Size gridMapSize = map.getSize();
  gridMapResolution_ = map.getResolution();
  // Faster access to grid map values.
  const grid_map::Matrix dataMap = map[inputLayer];
  // Height and width of submap.
  // Submap is Map without the outermost line of cells, no need to check if index is inside.
  const Index submapStartIndex(1, 1);
  const Index submapBufferSize(gridMapSize(0) - 2, gridMapSize(1) - 2);

  // For each cell in submap.
  for (SubmapIterator iterator(map, submapStartIndex, submapBufferSize); !iterator.isPastEnd();
    ++iterator)
  {
    const Index index(*iterator);
    rasterSingleNormalComputation(map, outputLayersPrefix, dataMap, index);
  }

  const double end = clock.now().seconds();
  RCLCPP_DEBUG_THROTTLE(
    this->logging_interface_->get_logger(), clock, 2.0, "NORMAL COMPUTATION TIME = %f",
    (end - start));
}

template<typename T>
void NormalVectorsFilter<T>::computeWithRasterParallel(
  GridMap & map,
  const std::string & inputLayer,
  const std::string & outputLayersPrefix)
{
  rclcpp::Clock clock;
  const double start = clock.now().seconds();

  const grid_map::Size gridMapSize = map.getSize();
  gridMapResolution_ = map.getResolution();
  // Faster access to grid map values if copy grid map layer into local matrix.
  const grid_map::Matrix dataMap = map[inputLayer];
  // Height and width of submap.
  // Submap is Map without the outermost line of cells, no need to check if index is inside.
  const Index submapStartIndex(1, 1);
  const Index submapBufferSize(gridMapSize(0) - 2, gridMapSize(1) - 2);
  if (submapBufferSize(1) != 0) {
    // Parallelized iteration through the map.
    tbb::parallel_for(
      0, submapBufferSize(0) * submapBufferSize(1), [&](int range) {
        const Index index(
          range / submapBufferSize(1) + submapStartIndex(0),
          range % submapBufferSize(1) + submapStartIndex(1));
        rasterSingleNormalComputation(map, outputLayersPrefix, dataMap, index);
      });
  } else {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "Grid map size is too small for normal raster computation");
  }

  const double end = clock.now().seconds();
  RCLCPP_DEBUG_THROTTLE(
    this->logging_interface_->get_logger(), clock, 2.0, "NORMAL COMPUTATION TIME = %f",
    (end - start));
}

template<typename T>
void NormalVectorsFilter<T>::rasterSingleNormalComputation(
  GridMap & map, const std::string & outputLayersPrefix,
  const grid_map::Matrix & dataMap, const grid_map::Index & index)
{
  // Inspiration for algorithm:
  // http://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
  const double centralCell = dataMap(index(0), index(1));
  double topCell = dataMap(index(0) - 1, index(1));
  double rightCell = dataMap(index(0), index(1) + 1);
  double bottomCell = dataMap(index(0) + 1, index(1));
  double leftCell = dataMap(index(0), index(1) - 1);

  // Neighboring cells configuration checked in X and Y direction independently.
  // Gridmap frame is defined rotated 90 degrees anticlockwise compared to
  // direction of matrix if we take rows as Y coordinate and columns as X coordinate.
  // In Y direction cell numbered as follows: left 0, center 1, right 2.
  // In X direction cell numbered as follows: top 0, center 1, bottom 2.
  // To find configuration we are in,
  // multiply value of map.isValid of cell in question, True or False,
  // by 2^(number of the cell).
  // Each configuration will have a different number associated with it and then use a switch.

  const int configurationDirX = 1 * static_cast<int>(std::isfinite(topCell)) + 2 *
    static_cast<int>(std::isfinite(centralCell)) +
    4 * static_cast<int>(std::isfinite(bottomCell));
  const int configurationDirY = 1 * static_cast<int>(std::isfinite(leftCell)) + 2 *
    static_cast<int>(std::isfinite(centralCell)) +
    4 * static_cast<int>(std::isfinite(rightCell));

  // If outer cell height value is missing use the central value, however
  // the formula for the normal calculation has to take into account that
  // the distance of the cells used for normal calculation is different.
  bool validConfiguration = true;
  double distanceX;
  switch (configurationDirX) {
    case 7:  // All 3 cell height values are valid.
      // Top and bottom cell centers are 2 cell resolution distant.
      distanceX = 2 * gridMapResolution_;
      break;
    case 6:  // Top cell height value not valid.
      topCell = centralCell;
      distanceX = gridMapResolution_;
      break;
    case 5:  // Central cell height value not valid. Not a problem.
      distanceX = 2 * gridMapResolution_;
      break;
    case 3:  // Bottom cell height value not valid.
      bottomCell = centralCell;
      distanceX = gridMapResolution_;
      break;

    default:
      // More than 1 cell height values are not valid,
      // normal vector will not be calculated in this location.
      validConfiguration = false;
  }

  double distanceY;
  switch (configurationDirY) {
    case 7:  // All 3 cell height values are valid.
      // Left and right cell centers are 2 call resolution distant.
      distanceY = 2 * gridMapResolution_;
      break;
    case 6:  // Left cell height value not valid.
      leftCell = centralCell;
      distanceY = gridMapResolution_;
      break;
    case 5:  // Central cell height value not valid. Not a problem.
      distanceY = 2 * gridMapResolution_;
      break;
    case 3:  // Right cell height value not valid.
      rightCell = centralCell;
      distanceY = gridMapResolution_;
      break;

    default:
      // More than 1 cell height values are not valid,
      // normal vector will not be calculated in this location.
      validConfiguration = false;
  }

  if (validConfiguration) {
    // Normal vector initialization
    Vector3 normalVector = Vector3::Zero();
    // X DIRECTION
    normalVector(0) = (bottomCell - topCell) / distanceX;
    // Y DIRECTION
    normalVector(1) = (rightCell - leftCell) / distanceY;
    // Z DIRECTION
    normalVector(2) = +1;

    normalVector.normalize();

    // Check direction of the normal vector and flip the sign towards the user defined direction.
    if (normalVector.dot(normalVectorPositiveAxis_) < 0.0) {
      normalVector = -normalVector;
    }

    map.at(outputLayersPrefix + "x", index) = normalVector.x();
    map.at(outputLayersPrefix + "y", index) = normalVector.y();
    map.at(outputLayersPrefix + "z", index) = normalVector.z();
  }
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::NormalVectorsFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
