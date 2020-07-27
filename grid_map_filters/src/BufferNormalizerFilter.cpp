/*
 * BufferNormalizerFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/BufferNormalizerFilter.hpp"

#include <grid_map_core/GridMap.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace grid_map
{

template<typename T>
BufferNormalizerFilter<T>::BufferNormalizerFilter()
{
}

template<typename T>
BufferNormalizerFilter<T>::~BufferNormalizerFilter()
{
}

template<typename T>
bool BufferNormalizerFilter<T>::configure()
{
  return true;
}

template<typename T>
bool BufferNormalizerFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();
  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::BufferNormalizerFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
