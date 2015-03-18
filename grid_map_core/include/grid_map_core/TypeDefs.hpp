/*
 * TypeDefs.hpp
 *
 *  Created on: March 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

// Eigen
#include <Eigen/Core>

#pragma once

namespace grid_map_core {

  typedef Eigen::MatrixXf Grid;
  typedef Eigen::Vector2d Position2;
  typedef Eigen::Vector3d Position3;
  typedef Eigen::Array2i Index;
  typedef Eigen::Array2i Size;
  typedef Eigen::Array2d Length;

} /* namespace */
