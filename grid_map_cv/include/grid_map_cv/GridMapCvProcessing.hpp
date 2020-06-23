/*
 * GridMapCvProcessing.hpp
 *
 *  Created on: Apr 15, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_CV__GRIDMAPCVPROCESSING_HPP_
#define GRID_MAP_CV__GRIDMAPCVPROCESSING_HPP_

#include <grid_map_core/grid_map_core.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

namespace grid_map
{

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

  /*!
   * Changes the resolution of a grid map with help of OpenCV's interpolation algorithms.
   * @param[in] gridMapSource the source grid map.
   * @param[out] gridMapResult the resulting grid map with the desired resolution.
   * @param[in] resolution the desired resolution.
   * @param[in](optional) interpolationAlgorithm the interpolation method.
   * @return true if successful, false otherwise.
   */
  static bool changeResolution(
    const grid_map::GridMap & gridMapSource,
    grid_map::GridMap & gridMapResult,
    const double resolution,
    const int interpolationAlgorithm = cv::INTER_CUBIC);
};

}  // namespace grid_map
#endif  // GRID_MAP_CV__GRIDMAPCVPROCESSING_HPP_
