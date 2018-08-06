/*
 * CurvatureFilter.hpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>

#include <Eigen/Core>
#include <string>

namespace grid_map {

/*!
 * Compute the curvature (second derivative) of a layer in the map.
 */
template<typename T>
class CurvatureFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  CurvatureFilter();

  /*!
   * Destructor.
   */
  virtual ~CurvatureFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Compute the curvature of a layer in a map and
   * saves it as additional grid map layer.
   * @param mapIn grid map containing the layer for which the curvature is computed for.
   * @param mapOut grid map containing mapIn and the new layer for the curvature.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

} /* namespace */
