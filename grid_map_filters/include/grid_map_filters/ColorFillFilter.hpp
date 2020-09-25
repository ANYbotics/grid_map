/*
 * ColorFillFilter.hpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__COLORFILLFILTER_HPP_
#define GRID_MAP_FILTERS__COLORFILLFILTER_HPP_

#include <filters/filter_base.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map
{

/*!
 * Creates a new color layer.
 */
template<typename T>
class ColorFillFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  ColorFillFilter();

  /*!
   * Destructor.
   */
  virtual ~ColorFillFilter();

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
  //! Color.
  double r_, g_, b_;

  //! Mask layer name.
  std::string maskLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__COLORFILLFILTER_HPP_
