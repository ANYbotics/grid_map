/*
 * CurvatureFilter.hpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <Eigen/Core>
#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

/*!
 * Compute the curvature (second derivative) of a layer in the map.
 */
class CurvatureFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  CurvatureFilter();

  /*!
   * Destructor.
   */
  ~CurvatureFilter() override;

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Compute the curvature of a layer in a map and
   * saves it as additional grid map layer.
   * @param mapIn grid map containing the layer for which the curvature is computed for.
   * @param mapOut grid map containing mapIn and the new layer for the curvature.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
