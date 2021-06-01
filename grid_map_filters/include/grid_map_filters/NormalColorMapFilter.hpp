/*
 * NormalColorMapFilter.hpp
 *
 *  Created on: Aug 22, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>

#include <Eigen/Core>
#include <string>

namespace grid_map {

/*!
 * Compute a new color layer based on normal vectors layers.
 */
template<typename T>
class NormalColorMapFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  NormalColorMapFilter();

  /*!
   * Destructor.
   */
  virtual ~NormalColorMapFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Compute a new color layer based on normal vectors layers.
   * @param mapIn grid map containing the layers of the normal vectors.
   * @param mapOut grid map containing mapIn and the new color layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! Input layers prefix.
  std::string inputLayersPrefix_;

  //! Output layer name.
  std::string outputLayer_;
};

} /* namespace */
