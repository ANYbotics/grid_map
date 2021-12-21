/*
 * LightIntensityFilter.hpp
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
 * Compute the diffuse lighting of a surface as new black and white color layer.
 */
class LightIntensityFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  LightIntensityFilter();

  /*!
   * Destructor.
   */
  ~LightIntensityFilter() override;

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Compute the diffuse lighting layer.
   * @param mapIn grid map containing the layers of the normal vectors.
   * @param mapOut grid map containing mapIn and the black and white lighting color layer.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! Input layers prefix.
  std::string inputLayersPrefix_;

  //! Output layer name.
  std::string outputLayer_;

  //! Light direction.
  Eigen::Vector3f lightDirection_;
};

}  // namespace grid_map
