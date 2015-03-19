/*
 * DeletionFilter.cpp
 *
 *  Created on: Mar 19, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/DeletionFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/GridMap.hpp>

namespace filters {

template<typename T>
DeletionFilter<T>::DeletionFilter()
{

}

template<typename T>
DeletionFilter<T>::~DeletionFilter()
{

}

template<typename T>
bool DeletionFilter<T>::configure()
{
  // Load Parameters
  if (!FilterBase<T>::getParam(std::string("deletionTypes"), delTypes_)) {
    ROS_ERROR("MinFilter did not find param deletionTypes");
    return false;
  }

  return true;
}

template<typename T>
bool DeletionFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;

  for (int i = 0; i < delTypes_.size(); i++) {
    // Check if layer exists.
    if (!mapOut.exists(delTypes_.at(i))) {
      ROS_ERROR("Check your deletion types! Type %s does not exist",
                delTypes_.at(i).c_str());
      continue;
    }

    if (!mapOut.remove(delTypes_.at(i))) {
      ROS_ERROR("Could not remove type %s", delTypes_.at(i).c_str());
    }
  }

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(DeletionFilter, filters::DeletionFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
