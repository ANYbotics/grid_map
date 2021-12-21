/*
 * DeletionFilter.hpp
 *
 *  Created on: Mar 19, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <string>
#include <vector>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

/*!
 * Deletion filter class deletes layers of a grid map.
 */
class DeletionFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  DeletionFilter();

  /*!
   * Destructor.
   */
  ~DeletionFilter() override;

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Deletes the specified layers of a grid map.
   * @param mapIn gridMap with the different layers.
   * @param mapOut gridMap without the deleted layers.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;

 private:
  //! List of layers that should be deleted.
  std::vector<std::string> layers_;
};

}  // namespace grid_map
