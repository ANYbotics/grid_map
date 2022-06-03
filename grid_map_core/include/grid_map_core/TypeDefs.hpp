/*
 * TypeDefs.hpp
 *
 *  Created on: March 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// Eigen
#pragma once

#include <Eigen/Core>

namespace grid_map {

  using Matrix = Eigen::MatrixXf;
  using DataType = Matrix::Scalar;
  using Position = Eigen::Vector2d;
  using Vector = Eigen::Vector2d;
  using Position3 = Eigen::Vector3d;
  using Vector3 = Eigen::Vector3d;
  using Index = Eigen::Array2i;
  using Size = Eigen::Array2i;
  using Length = Eigen::Array2d;
  using Time = uint64_t;

  /*
   * Interpolations are ordered in the order
   * of increasing accuracy and computational complexity.
   * INTER_NEAREST - fastest, but least accurate,
   * INTER_CUBIC - slowest, but the most accurate.
   * see:
   * https://en.wikipedia.org/wiki/Bicubic_interpolation
   * https://web.archive.org/web/20051024202307/http://www.geovista.psu.edu/sites/geocomp99/Gc99/082/gc_082.htm
   * for more info. Cubic convolution algorithm is also known as piecewise cubic
   * interpolation and in general does not guarantee continuous
   * first derivatives.
   */
  enum class InterpolationMethods{
      INTER_NEAREST, // nearest neighbor interpolation
      INTER_LINEAR,   // bilinear interpolation
      INTER_CUBIC_CONVOLUTION, //piecewise bicubic interpolation using convolution algorithm
      INTER_CUBIC // standard bicubic interpolation
  };

}  // namespace grid_map

