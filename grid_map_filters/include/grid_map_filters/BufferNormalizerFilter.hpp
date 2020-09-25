/*
 * BufferNormalizerFilter.hpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__BUFFERNORMALIZERFILTER_HPP_
#define GRID_MAP_FILTERS__BUFFERNORMALIZERFILTER_HPP_

#include <filters/filter_base.hpp>

#include <string>

namespace grid_map
{

/*!
 * Normalizes the buffer of a map such that it has default (zero) start index.
 */
template<typename T>
class BufferNormalizerFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  BufferNormalizerFilter();

  /*!
   * Destructor.
   */
  virtual ~BufferNormalizerFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Normalizes the buffer of a map.
   * @param mapIn the input map before normalization.
   * @param mapOut the normalized map.
   */
  bool update(const T & mapIn, T & mapOut) override;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__BUFFERNORMALIZERFILTER_HPP_
