/*
 * InpaintFilter.cpp
 *
 *  Created on: May 6, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "filters/InpaintFilter.hpp"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

using namespace filters;

namespace grid_map_filters {

template<typename T>
InpaintFilter<T>::InpaintFilter()
    : radius_(5.0),
      inputLayer_("elevation"),
      type_("inpaint") {

}

template<typename T>
InpaintFilter<T>::~InpaintFilter() {

}

template<typename T>
bool InpaintFilter<T>::configure() {
  if (!FilterBase < T > ::getParam(std::string("radius"), radius_)) {
    ROS_ERROR("InpaintRadius filter did not find param radius.");
    return false;
  }

  if (radius_ < 0.0) {
    ROS_ERROR("Radius must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Radius = %f.", radius_);

  if (!FilterBase < T > ::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Inpaint filter did not find param input_layer.");
    return false;
  }

  ROS_DEBUG("Inpaint input layer is = %s.", inputLayer_.c_str());

  if (!FilterBase < T > ::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("Inpaint filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("Inpaint map type = %s.", type_.c_str());

  return true;
}

template<typename T>
bool InpaintFilter<T>::update(const T& mapIn, T& mapOut) {
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  //Convert elevation layer to OpenCV image to fill in holes
  //Get the inpaint mask (nonzero pixels indicate where values need to be filled in)
  mapOut.add("inpaintMask", 0.0);

  mapOut.setBasicLayers(std::vector<std::string>());
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, inputLayer_)) {
      mapOut.at("inpaintMask", *iterator) = 1.0;
    }
  }
  cv::Mat originalImage;
  cv::Mat mask;
  cv::Mat filledImage;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<unsigned char, 3>(mapOut, inputLayer_, CV_8UC3, minValue, maxValue,
                                                          originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "inpaintMask", CV_8UC1, mask);

  cv::inpaint(originalImage, mask, filledImage, radius_, cv::INPAINT_NS);  //INPAINT_TELEA

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, type_, mapOut, minValue, maxValue);
  //mapOut.erase("inpaintMask");

  return true;
}

}/* namespace */

PLUGINLIB_REGISTER_CLASS(InpaintFilter, grid_map_filters::InpaintFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
