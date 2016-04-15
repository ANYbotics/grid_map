/*
 * GridMapCvProcessing.hpp
 *
 *  Created on: Apr 15, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>

// OpenCV
#include <opencv/cv.h>

namespace grid_map {

/*!
 * Processing of grid maps with OpenCV methods.
 */
class GridMapCvProcessing
{
 public:
  /*!
   * Default constructor.
   */
  GridMapCvProcessing();

  /*!
   * Destructor.
   */
  virtual ~GridMapCvProcessing();

  static bool changeResolution(const grid_map::GridMap& gridMapSource,
                               grid_map::GridMap& gridMapResult,
                               const double resolution,
                               const int interpolationAlgrithm = cv::INTER_CUBIC);

};

} /* namespace */
