/*
 * TypeDefs.hpp
 *
 *  Created on: March 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_CORE__TYPEDEFS_HPP_
#define GRID_MAP_CORE__TYPEDEFS_HPP_

// Eigen
#include <Eigen/Core>

namespace grid_map
{

typedef Eigen::MatrixXf Matrix;
typedef Matrix::Scalar DataType;
typedef Eigen::Vector2d Position;
typedef Eigen::Vector2d Vector;
typedef Eigen::Vector3d Position3;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Array2i Index;
typedef Eigen::Array2i Size;
typedef Eigen::Array2d Length;
typedef uint64_t Time;

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
enum class InterpolationMethods
{
  INTER_NEAREST,  // nearest neighbor interpolation
  INTER_LINEAR,  // bilinear interpolation
  INTER_CUBIC_CONVOLUTION,  // piecewise bicubic interpolation using convolution algorithm
  INTER_CUBIC  // standard bicubic interpolation
};

}  // namespace grid_map
#endif  // GRID_MAP_CORE__TYPEDEFS_HPP_
