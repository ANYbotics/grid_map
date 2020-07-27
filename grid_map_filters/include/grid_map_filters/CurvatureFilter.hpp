/*
 * CurvatureFilter.hpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__CURVATUREFILTER_HPP_
#define GRID_MAP_FILTERS__CURVATUREFILTER_HPP_

#include <filters/filter_base.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map
{

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
  bool configure() override;

  /*!
   * Compute the curvature of a layer in a map and
   * saves it as additional grid map layer.
   * @param mapIn grid map containing the layer for which the curvature is computed for.
   * @param mapOut grid map containing mapIn and the new layer for the curvature.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__CURVATUREFILTER_HPP_
