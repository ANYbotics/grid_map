/*
 * ColorBlendingFilter.hpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__COLORBLENDINGFILTER_HPP_
#define GRID_MAP_FILTERS__COLORBLENDINGFILTER_HPP_

#include <filters/filter_base.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map
{

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
  bool configure() override;

  /*!
   * Compute a new color layer based on blending two color layers.
   * @param mapIn grid map containing the two color layers.
   * @param mapOut grid map containing mapIn and the blended color layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  enum class BlendModes
  {
    Normal,
    HardLight,
    SoftLight
  };

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
#endif  // GRID_MAP_FILTERS__COLORBLENDINGFILTER_HPP_
