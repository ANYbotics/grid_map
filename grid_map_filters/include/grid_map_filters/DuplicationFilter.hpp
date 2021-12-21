/*
 * DuplicationFilter.hpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
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
class DuplicationFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  DuplicationFilter();

  /*!
   * Destructor.
   */
  ~DuplicationFilter() override;

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Duplicates the specified layers of a grid map.
   * @param mapIn with the layer to duplicate.
   * @param mapOut with the layer duplicated.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! Name of the layer that is duplicated.
  std::string inputLayer_;

  //! Name of the new layer.
  std::string outputLayer_;
};

}  // namespace grid_map
