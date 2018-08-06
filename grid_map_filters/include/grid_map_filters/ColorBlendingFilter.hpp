/*
 * ColorBlendingFilter.hpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>

#include <Eigen/Core>
#include <string>

namespace grid_map {

/*!
 * Blend two color layers.
 */
template<typename T>
class ColorBlendingFilter : public filters::FilterBase<T>
{
 public:
  /*!
   * Constructor
   */
  ColorBlendingFilter();

  /*!
   * Destructor.
   */
  virtual ~ColorBlendingFilter();

  /*!
   * Configures the filter.
   */
  virtual bool configure();

  /*!
   * Compute a new color layer based on blending two color layers.
   * @param mapIn grid map containing the two color layers.
   * @param mapOut grid map containing mapIn and the blended color layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  enum class BlendModes {
    Normal,
    HardLight,
    SoftLight
  };

  //! Input layers.
  std::string backgroundLayer_, foregroundLayer_;

  //! Blend mode.
  BlendModes blendMode_;

  //! Opacity of foreground layer.
  double opacity_;

  //! Output layer name.
  std::string outputLayer_;
};

} /* namespace */
