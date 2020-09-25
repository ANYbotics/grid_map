/*
 * SetBasicLayersFilters.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__SETBASICLAYERSFILTER_HPP_
#define GRID_MAP_FILTERS__SETBASICLAYERSFILTER_HPP_

#include <filters/filter_base.hpp>

#include <vector>
#include <string>

namespace grid_map
{

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
  bool configure() override;

  /*!
   * Set the specified layers as basic layers.
   * @param mapIn input grid map.
   * @param mapOut output grid map with basic layers set.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! List of layers that should be set as basic layers.
  std::vector<std::string> layers_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__SETBASICLAYERSFILTER_HPP_
