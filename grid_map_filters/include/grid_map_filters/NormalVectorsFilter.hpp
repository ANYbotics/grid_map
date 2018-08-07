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
template<typename T>
class NormalVectorsFilter : public filters::FilterBase<T>
{

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

  void computeWithArea(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);
  void computeWithRaster(GridMap& map, const std::string& inputLayer, const std::string& outputLayersPrefix);

  enum class Method {
    Area,
    Raster
  };

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

} /* namespace */
