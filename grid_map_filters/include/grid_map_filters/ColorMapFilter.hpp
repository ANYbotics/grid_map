/*
 * ColorMapFilter.hpp
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
#include <grid_map_core/TypeDefs.hpp>

namespace grid_map {

/*!
 * Creates a new color layer with the color mapped between min. and max. value.
 */
class ColorMapFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  ColorMapFilter();

  /*!
   * Destructor.
   */
  ~ColorMapFilter() override;

  /*!
   * Configures the filter.
   */
  bool configure() override;

  /*!
   * Adds a new color layer.
   * @param mapIn grid map to add the new layer.
   * @param mapOut grid map the grid map with the new color layer.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! Min./max. colors.
  Eigen::Vector3f minColor_, maxColor_;

  //! Min./max. values.
  double min_, max_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
