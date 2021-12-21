/*
 * SetBasicLayersFilters.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <string>
#include <vector>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

/*!
 * Set specified layers of a grid map as basic layers.
 */
class SetBasicLayersFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  SetBasicLayersFilter();

  /*!
   * Destructor.
   */
  ~SetBasicLayersFilter() override;

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Set the specified layers as basic layers.
   * @param mapIn input grid map.
   * @param mapOut output grid map with basic layers set.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! List of layers that should be set as basic layers.
  std::vector<std::string> layers_;
};

}  // namespace grid_map
