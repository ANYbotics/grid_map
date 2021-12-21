/*
 * MockFilter.hpp
 *
 *  Created on: Sep 24, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

/*!
 * Duplication filter class duplicates a layer of a grid map.
 */
class MockFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  MockFilter();

  /*!
   * Destructor.
   */
  ~MockFilter() override;

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Copies the input to the output. The time for the update is specified by processingTime_. Optionally the update is logged.
   * @param mapIn Input.
   * @param mapOut Output.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! Flag indicating wheter to also log on update.
  bool printName_{false};

  //! The time [ms] that the update function takes.
  uint processingTime_{0};
};

}  // namespace grid_map
