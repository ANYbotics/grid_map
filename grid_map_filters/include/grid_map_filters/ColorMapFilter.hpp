/*
 * ColorMapFilter.hpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__COLORMAPFILTER_HPP_
#define GRID_MAP_FILTERS__COLORMAPFILTER_HPP_

#include <grid_map_core/TypeDefs.hpp>
#include <filters/filter_base.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map
{

/*!
 * Creates a new color layer with the color mapped between min. and max. value.
 */
template<typename T>
class ColorMapFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  ColorMapFilter();

  /*!
   * Destructor.
   */
  virtual ~ColorMapFilter();

  /*!
   * Configures the filter.
   */
  bool configure() override;

  /*!
   * Adds a new color layer.
   * @param mapIn grid map to add the new layer.
   * @param mapOut grid map the grid map with the new color layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

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
#endif  // GRID_MAP_FILTERS__COLORMAPFILTER_HPP_
