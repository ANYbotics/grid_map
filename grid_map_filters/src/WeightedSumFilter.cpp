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
#include <Eigen/Dense>

namespace filters {

template<typename T>
WeightedSumFilter<T>::WeightedSumFilter()
      : typeOut_("traversability"),
        normalize_(1)
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
  if (!FilterBase<T>::getParam(std::string("typeOut"), typeOut_)) {
    ROS_ERROR("WeightedSumFilter did not find param typeOut");
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

  if (!FilterBase<T>::getParam(std::string("normalize"), normalize_)) {
    ROS_ERROR("WeightedSumFilter did not find param normalize");
    return false;
  }

  if (normalize_) {
    // Normalize weights
      double sumWeights = 0.0;

      for (std::vector<double>::iterator it=additionWeights_.begin(); it!=additionWeights_.end(); ++it) {
        sumWeights += *it;
      }

      for (std::vector<double>::iterator it=additionWeights_.begin(); it!=additionWeights_.end(); ++it) {
        *it /= sumWeights;
      }
  }
  else {
    ROS_WARN("No normalization of the weights");
  }

  return true;
}

template<typename T>
bool WeightedSumFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(typeOut_, mapIn.get("elevation"));
  bool hasSum = false;

  Eigen::MatrixXf sum, newSummand;

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
      newSummand = additionWeights_.at(i)*mapOut.get(additionTypes_.at(i));
      if (typeOut_ != "traversability") {
        sum += newSummand;
      }
      else {
        // Mark the addition of a non-traversable cell as non-traversable (set to zero)
        // Iterate through the rows
        for (int j=0; j<sum.rows(); j++) {
          // Iterate through the cols
          for (int k=0; k<sum.cols(); k++) {
            // Check if there are non-traversable cells
            if (sum(j,k) == 0.0 || newSummand(j,k) == 0.0) {
              sum(j,k) = 0.0;
            }
            else {
              sum(j,k) += newSummand(j,k);
            }
          }
        }
      }
    }

  }
  mapOut.add(typeOut_, sum);

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(WeightedSumFilter, filters::WeightedSumFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
