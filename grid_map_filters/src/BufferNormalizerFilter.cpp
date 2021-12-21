/*
 * BufferNormalizerFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/BufferNormalizerFilter.hpp"

#include <grid_map_core/GridMap.hpp>

using namespace filters;

namespace grid_map {

BufferNormalizerFilter::BufferNormalizerFilter() = default;

BufferNormalizerFilter::~BufferNormalizerFilter() = default;

bool BufferNormalizerFilter::configure() {
  return true;
}

bool BufferNormalizerFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  return true;
}

}  // namespace grid_map
