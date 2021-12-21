/*
 * NormalColorMapFilter.hpp
 *
 *  Created on: Aug 22, 2017
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
 * Compute a new color layer based on normal vectors layers.
 */
class NormalColorMapFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  NormalColorMapFilter();

  /*!
   * Destructor.
   */
  ~NormalColorMapFilter() override;

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Compute a new color layer based on normal vectors layers.
   * @param mapIn grid map containing the layers of the normal vectors.
   * @param mapOut grid map containing mapIn and the new color layer.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! Input layers prefix.
  std::string inputLayersPrefix_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
