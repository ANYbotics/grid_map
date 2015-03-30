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
#include <grid_map_core/GridMap.hpp>

using namespace filters;

namespace grid_map_filters {

template<typename T>
WeightedSumFilter<T>::WeightedSumFilter()
      : layerOut_("traversability"),
        normalize_(true)
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
  if (!FilterBase<T>::getParam(std::string("layer_out"), layerOut_)) {
    ROS_ERROR("WeightedSumFilter did not find parameter 'layer_out'.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("layers"), layers_)) {
    ROS_ERROR("WeightedSumFilter did not find parameter 'layers'");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("weights"), weights_)) {
    ROS_ERROR("WeightedSumFilter did not find parameter 'weights'.");
    return false;
  }

  if (layers_.size() != weights_.size()) {
    ROS_ERROR("Number of weights must correspond number of layers!");
    return false;
  }

  int normalizeParameter;
  if (!FilterBase<T>::getParam(std::string("normalize"), normalizeParameter)) {
    ROS_ERROR("WeightedSumFilter did not find parameter 'normalize'");
    return false;
  }
  normalize_ = normalizeParameter==1 ? true : false;

  if (normalize_) {
    // Normalize weights.
    double sumWeights = 0.0;

    for (std::vector<double>::iterator it=weights_.begin(); it!=weights_.end(); ++it) {
      sumWeights += *it;
    }

    for (std::vector<double>::iterator it=weights_.begin(); it!=weights_.end(); ++it) {
      *it /= sumWeights;
    }
  }
  else {
    ROS_WARN("No normalization of the weights.");
  }

  return true;
}

template<typename T>
bool WeightedSumFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(layerOut_);
  bool hasSum = false;

  grid_map::Matrix sum, newSummand;

  int i = 0;
  for (const auto& layer : layers_) {
    // Check if layer exists.
    if (!mapOut.exists(layer)) {
      ROS_ERROR("Check your addition layers! Layer '%s' does not exist.", layer.c_str());
      return false;
    }

    if (!hasSum) {
      sum = weights_.at(i) * mapOut.get(layer);
      hasSum = true;
    }
    else {
      newSummand = weights_.at(i) * mapOut.get(layer);
      if (layerOut_ != "traversability") {
        sum += newSummand;
      } else {
        // Mark the addition of a non-traversable cell as non-traversable (set to zero)
        // Iterate through the rows
        for (int j = 0; j < sum.rows(); j++) {
          // Iterate through the cols
          for (int k = 0; k < sum.cols(); k++) {
            // Check if there are non-traversable cells
            if (sum(j, k) == 0.0 || newSummand(j, k) == 0.0) {
              sum(j, k) = 0.0;
            } else {
              sum(j, k) += newSummand(j, k);
            }
          }
        }
      }
    }
    ++i;
  }
  mapOut.add(layerOut_, sum);

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(WeightedSumFilter, grid_map_filters::WeightedSumFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
