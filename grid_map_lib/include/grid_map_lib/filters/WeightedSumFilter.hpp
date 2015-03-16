/*
 * WeightedSumFilter.hpp
 *
 *  Created on: Mar 16, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef WEIGHTEDSUMFILTER_HPP
#define WEIGHTEDSUMFILTER_HPP
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include <filters/filter_base.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/iterators/GridMapIterator.hpp>

namespace filters {

/*!
 * Slope Filter class to compute the slope traversability value of an elevation map.
 */
template<typename T>
class WeightedSumFilter : public FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  WeightedSumFilter();

  /*!
   * Destructor.
   */
  virtual ~WeightedSumFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes the weighted sum of different layers of traversability maps.
   * @param elevationMap the map for which the slope traversability value is computed.
   * @param traversability_map gridMap with total traversability value.
   */
  virtual bool update(const T & elevation_map, T& traversability_map);

 private:
};

} /* namespace */

#endif
