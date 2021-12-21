/*
 * MinInRadiusFilter.hpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

/*!
 * Filter class to compute the minimal value inside a radius.
 */
class MinInRadiusFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor.
   */
  MinInRadiusFilter();

  /*!
   * Destructor.
   */
  ~MinInRadiusFilter() override;

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Computes for each value in the input layer the minimum of all values in a radius around it.
   * Saves this minimal value in an additional output layer.
   * @param mapIn grid map containing the input layer.
   * @param mapOut grid map containing the original layers and the new layer with the minimal values.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! Radius to take the minimum in.
  double radius_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
