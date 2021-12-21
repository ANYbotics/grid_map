/*
 * ColorFillFilter.hpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <Eigen/Core>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

/*!
 * Creates a new color layer.
 */
class ColorFillFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  ColorFillFilter();

  /*!
   * Destructor.
   */
  ~ColorFillFilter() override;

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
  //! Color.
  double r_, g_, b_;

  //! Mask layer name.
  std::string maskLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
