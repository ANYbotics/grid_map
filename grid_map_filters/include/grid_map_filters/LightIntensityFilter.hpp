/*
 * LightIntensityFilter.hpp
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
 * Compute the diffuse lighting of a surface as new black and white color layer.
 */
template<typename T>
class LightIntensityFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  LightIntensityFilter();

  /*!
   * Destructor.
   */
  virtual ~LightIntensityFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Compute the diffuse lighting layer.
   * @param mapIn grid map containing the layers of the normal vectors.
   * @param mapOut grid map containing mapIn and the black and white lighting color layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! Input layers prefix.
  std::string inputLayersPrefix_;

  //! Output layer name.
  std::string outputLayer_;

  //! Light direction.
  Eigen::Vector3f lightDirection_;
};

} /* namespace */
