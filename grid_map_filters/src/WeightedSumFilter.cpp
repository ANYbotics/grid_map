/*
 * WeightedSumFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/WeightedSumFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/GridMap.hpp>

// Eigen
#include <Eigen/Core>

namespace filters {

template<typename T>
WeightedSumFilter<T>::WeightedSumFilter()
      : sumType_("traversability")
{

}

template<typename T>
WeightedSumFilter<T>::~WeightedSumFilter()
{

}

template<typename T>
bool WeightedSumFilter<T>::configure()
{
  // Load Parameters
  if (!FilterBase<T>::getParam(std::string("sumType"), sumType_)) {
    ROS_ERROR("WeightedSumFilter did not find param sumType");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("filterTypes"), additionTypes_)) {
    ROS_ERROR("WeightedSumFilter did not find param filterTypes");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("filterWeights"), additionWeights_)) {
    ROS_ERROR("WeightedSumFilter did not find param filterWeights");
    return false;
  }

  if (additionTypes_.size() != additionWeights_.size()) {
    ROS_ERROR("Number of weights must correspond number of layers!");
    return false;
  }

  // Normalize weights
  double sumWeights = 0.0;
  for (std::vector<double>::iterator it=additionWeights_.begin(); it!=additionWeights_.end(); ++it) {
//    ROS_INFO("Weight = %f", *it);
    sumWeights += *it;
  }

//  ROS_INFO("Sum weights = %f", sumWeights);
  for (std::vector<double>::iterator it=additionWeights_.begin(); it!=additionWeights_.end(); ++it) {
    *it /= sumWeights;
//    ROS_INFO("Weight = %f", *it);
  }
  return true;
}

template<typename T>
bool WeightedSumFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(sumType_, mapIn.get("elevation"));
  bool hasSum = false;

  Eigen::MatrixXf sum;

  for (int i=0; i<additionTypes_.size(); i++) {
    if (!mapOut.exists(additionTypes_.at(i))) {
      ROS_ERROR("Check your addition types! Type %s does not exist",additionTypes_.at(i).c_str());
      return false;
    }

    if (!hasSum) {
      sum = additionWeights_.at(i)*mapOut.get(additionTypes_.at(i));
      hasSum = true;

    }
    else {
      sum += additionWeights_.at(i)*mapOut.get(additionTypes_.at(i));
    }

  }
  mapOut.add(sumType_, sum);

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(WeightedSumFilter, filters::WeightedSumFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
