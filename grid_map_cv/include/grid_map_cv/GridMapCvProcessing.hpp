/*
 * GridMapCvProcessing.hpp
 *
 *  Created on: Apr 15, 2016
 *      Author: Péter Fankhauser, Magnus Gärtner
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>

// OpenCV
#include <cv_bridge/cv_bridge.h>

namespace grid_map {

/*!
 * Processing of grid maps with OpenCV methods.
 */
class GridMapCvProcessing {
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
  static bool changeResolution(const GridMap& gridMapSource, GridMap& gridMapResult, const double resolution,
                               const int interpolationAlgorithm = cv::INTER_CUBIC);

  /*!
   * Apply isometric transformation (rotation + offset) to grid map and returns the transformed map.
   * Note: The returned map may not have the same length since it's geometric description contains
   * the original map.
   * Also note that the map treats the height layer differently from other layers. I.e, the translation in z-direction is applied only to
   * the height layer. For layers other than the height layer that contain geometric information this function might compute unexpected
   * results. E.g transforming the layers normals_x/y/z will only represent the same values at the transformed location but will not
   * transform the normal vectors into the new coordinate system.
   *
   * @param[consume] gridMapSource The map to transform.
   * @param[in] transform the requested transformation to apply.
   * @param[in] heightLayerName the height layer of the map.
   * @param[in] newFrameId frame index of the new map.
   * @return transformed map.
   * @throw std::out_of_range if no map layer with name `heightLayerName` is present.
   * @throw std::invalid_argument if the transform is not approximately z-aligned.
   */
  static GridMap getTransformedMap(GridMap&& gridMapSource, const Eigen::Isometry3d& transform, const std::string& heightLayerName,
                                   const std::string& newFrameId);
};

}  // namespace grid_map
