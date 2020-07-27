/*
 * MeanInRadiusFilter.hpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__MEANINRADIUSFILTER_HPP_
#define GRID_MAP_FILTERS__MEANINRADIUSFILTER_HPP_

#include <filters/filter_base.hpp>

#include <vector>
#include <string>

namespace grid_map
{

/*!
 * Filter class to find the mean of the values inside a radius.
 */
template<typename T>
class MeanInRadiusFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  MeanInRadiusFilter();

  /*!
   * Destructor.
   */
  virtual ~MeanInRadiusFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Computes for each value in the input layer the mean of all values in a radius around it
   * Saves this mean in an additional output layer.
   * @param mapIn grid map containing the input layer.
   * @param mapOut grid map containing the layers of the input map and the new layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Radius to take the mean from.
  double radius_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__MEANINRADIUSFILTER_HPP_
