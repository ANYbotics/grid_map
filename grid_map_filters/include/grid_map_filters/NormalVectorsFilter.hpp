/*
 * NormalVectorsFilter.hpp
 *
 *  Created on: May 05, 2015
 *      Author: Peter Fankhauser, Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>
#include <grid_map_core/grid_map_core.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map {

/*!
 * Compute the normal vectors of a layer in a map.
 */
template <typename T>
class NormalVectorsFilter : public filters::FilterBase<T> {
 public:
  /*!
   * Constructor
   */
  NormalVectorsFilter();

  /*!
   * Destructor.
   */
  virtual ~NormalVectorsFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Compute the normal vectors of a layer in a map and
   * saves it as additional grid map layer.
   * @param mapIn grid map containing the layer for which the normal vectors are computed for.
   * @param mapOut grid map containing mapIn and the new layers for the normal vectors.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  /*!
   * Estimate the normal vector at each point of the input layer by using points within a circle of specified radius.
   *
   * The eigen decomposition of the covariance matrix (3x3) of all data points is used to establish the normal direction.
   * Four cases can be identified when the eigenvalues are ordered in ascending order:
   *    1) The data is in a cloud -> all eigenvalues are non-zero
   *    2) The data is on a plane -> The first eigenvalue is zero
   *    3) The data is on a line -> The first two eigenvalues are zero.
   *    4) The data is in one point -> All eigenvalues are zero
   *
   * Only case 1 & 2 provide enough information the establish a normal direction.
   * The degenerate cases (3 or 4) are identified by checking if the second eigenvalue is zero.
   *
   * The numerical threshold (1e-8) for the eigenvalue being zero is given by the accuracy of the decomposition, as reported by Eigen:
   * https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
   *
   * Finally, the sign normal vector is correct to be in the same direction as the user defined "normal vector positive axis"
   *
   * @param map: grid map containing the layer for which the normal vectors are computed for.
   * @param inputLayer: Layer the normal vector should be computed for.
   * @param outputLayersPrefix: Output layer name prefix.
   */
  void computeWithArea(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);

  void computeWithRaster(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);

  enum class Method { Area, Raster };

  Method method_;

  //! Radius of submap for normal vector estimation.
  double estimationRadius_;

  //! Normal vector positive axis.
  Eigen::Vector3d normalVectorPositiveAxis_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayersPrefix_;
};

}  // namespace grid_map
