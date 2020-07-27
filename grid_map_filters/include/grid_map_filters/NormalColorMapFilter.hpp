/*
 * NormalColorMapFilter.hpp
 *
 *  Created on: Aug 22, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__NORMALCOLORMAPFILTER_HPP_
#define GRID_MAP_FILTERS__NORMALCOLORMAPFILTER_HPP_

#include <filters/filter_base.hpp>

#include <Eigen/Core>
#include <string>

namespace grid_map
{

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
  bool configure() override;

  /*!
   * Compute a new color layer based on normal vectors layers.
   * @param mapIn grid map containing the layers of the normal vectors.
   * @param mapOut grid map containing mapIn and the new color layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Input layers prefix.
  std::string inputLayersPrefix_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__NORMALCOLORMAPFILTER_HPP_
