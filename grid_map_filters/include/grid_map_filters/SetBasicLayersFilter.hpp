/*
 * SetBasicLayersFilters.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>

#include <vector>
#include <string>

namespace grid_map {

/*!
 * Set specified layers of a grid map as basic layers.
 */
template<typename T>
class SetBasicLayersFilter : public filters::FilterBase<T>
{
 public:
  /*!
   * Constructor
   */
  SetBasicLayersFilter();

  /*!
   * Destructor.
   */
  virtual ~SetBasicLayersFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Set the specified layers as basic layers.
   * @param mapIn input grid map.
   * @param mapOut output grid map with basic layers set.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! List of layers that should be set as basic layers.
  std::vector<std::string> layers_;

};

} /* namespace */
