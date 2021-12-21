/*
 * MockFilter.cpp
 *
 *  Created on: Sep 24, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_filters/MockFilter.hpp"

#include <chrono>
#include <thread>

#include <grid_map_core/GridMap.hpp>

using namespace filters;

namespace grid_map {

MockFilter::MockFilter() = default;

MockFilter::~MockFilter() = default;

bool MockFilter::configure() {
  if (!FilterBase::getParam(std::string("processing_time"), processingTime_)) {
    ROS_ERROR("MockFilter did not find parameter 'processing_time'.");
    return false;
  }

  if (!FilterBase::getParam(std::string("print_name"), printName_)) {
    ROS_INFO("MockFilter did not find parameter 'print_name'. Not printing the name. ");
    printName_ = false;
  }

  return true;
}

bool MockFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  if (printName_) {
    ROS_INFO_STREAM(this->getName() << ": update()");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(processingTime_));
  return true;
}

}  // namespace grid_map
