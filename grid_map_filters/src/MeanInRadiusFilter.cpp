/*
 * MeanInRadiusFilter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MeanInRadiusFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

template<typename T>
MeanInRadiusFilter<T>::MeanInRadiusFilter()
    : radius_(0.0)
{
}

template<typename T>
MeanInRadiusFilter<T>::~MeanInRadiusFilter()
{
}

template<typename T>
bool MeanInRadiusFilter<T>::configure()
{
  if (!FilterBase < T > ::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("MeanInRadius filter did not find parameter `radius`.");
    return false;
  }

  if (radius_ < 0.0) {
    ROS_ERROR("MeanInRadius filter: Radius must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Radius = %f.", radius_);

  if (!FilterBase < T > ::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("MeanInRadius filter did not find parameter `input_layer`.");
    return false;
  }

  ROS_DEBUG("MeanInRadius input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase < T > ::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("MeanInRadius filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("MeanInRadius output_layer = %s.", outputLayer_.c_str());
  return true;
}

template<typename T>
bool MeanInRadiusFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layers to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  double value;

  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {

    double valueSum = 0.0;
    int counter = 0;
    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Find the mean in a circle around the center
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius_); !submapIterator.isPastEnd();
        ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, inputLayer_))
        continue;
      value = mapOut.at(inputLayer_, *submapIterator);
      valueSum += value;
      counter++;
    }

    if (counter != 0)
      mapOut.at(outputLayer_, *iterator) = valueSum / counter;
  }

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(grid_map::MeanInRadiusFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
