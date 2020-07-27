/*
 * LightIntensityFilter.hpp
 *
 *  Created on: Sep 23, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__LIGHTINTENSITYFILTER_HPP_
#define GRID_MAP_FILTERS__LIGHTINTENSITYFILTER_HPP_

#include <filters/filter_base.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map
{

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
  bool configure() override;

  /*!
   * Compute the diffuse lighting layer.
   * @param mapIn grid map containing the layers of the normal vectors.
   * @param mapOut grid map containing mapIn and the black and white lighting color layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Input layers prefix.
  std::string inputLayersPrefix_;

  //! Output layer name.
  std::string outputLayer_;

  //! Light direction.
  Eigen::Vector3f lightDirection_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__LIGHTINTENSITYFILTER_HPP_
