/*
 * ColorFillFilter.hpp
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
  virtual bool configure();

  /*!
   * Adds a new color layer.
   * @param mapIn grid map to add the new layer.
   * @param mapOut grid map the grid map with the new color layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! Color.
  double r_, g_, b_;

  //! Mask layer name.
  std::string maskLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

} /* namespace */
