/*
 * MinFilter.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/MinFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/GridMap.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

namespace filters {

template<typename T>
MinFilter<T>::MinFilter()
    : typeOut_("traversability")
{

}

template<typename T>
MinFilter<T>::~MinFilter()
{

}

template<typename T>
bool MinFilter<T>::configure()
{
  // Load Parameters
  if (!FilterBase<T>::getParam(std::string("type_out"), typeOut_)) {
    ROS_ERROR("MinFilter did not find param type_out");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("filter_types"), minTypes_)) {
    ROS_ERROR("MinFilter did not find param filter_types");
    return false;
  }

  return true;
}

template<typename T>
bool MinFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(typeOut_, mapIn.get("elevation"));
  bool hasMin = false;

  Eigen::MatrixXf minMatrix;

  for (int i = 0; i < minTypes_.size(); i++) {
    // Check if layer exists.
    if (!mapOut.exists(minTypes_.at(i))) {
      ROS_ERROR("Check your min types! Type %s does not exist",
                minTypes_.at(i).c_str());
      return false;
    }

    if (!hasMin) {
      minMatrix = mapOut.get(minTypes_.at(i));
      hasMin = true;
    } else {
      minMatrix = minMatrix.cwiseMin(mapOut.get(minTypes_.at(i)));
    }
  }
  mapOut.add(typeOut_, minMatrix);

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(MinFilter, filters::MinFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
