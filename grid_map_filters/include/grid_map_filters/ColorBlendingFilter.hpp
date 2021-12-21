/*
 * ColorBlendingFilter.hpp
 *
 *  Created on: Sep 14, 2017
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
 * Blend two color layers.
 */
class ColorBlendingFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  ColorBlendingFilter();

  /*!
   * Destructor.
   */
  ~ColorBlendingFilter() override;

  /*!
   * Configures the filter.
   */
  bool configure() override;

  /*!
   * Compute a new color layer based on blending two color layers.
   * @param mapIn grid map containing the two color layers.
   * @param mapOut grid map containing mapIn and the blended color layer.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  enum class BlendModes { Normal, HardLight, SoftLight };

  //! Input layers.
  std::string backgroundLayer_, foregroundLayer_;

  //! Opacity of foreground layer.
  double opacity_;

  //! Blend mode.
  BlendModes blendMode_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
