/*
 * Functors.hpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_CORE__EIGEN_PLUGINS__FUNCTORS_HPP_
#define GRID_MAP_CORE__EIGEN_PLUGINS__FUNCTORS_HPP_

namespace grid_map
{

template<typename Scalar>
struct Clamp
{
  Clamp(const Scalar & min, const Scalar & max)
  : min_(min),
    max_(max)
  {
  }
  const Scalar operator()(const Scalar & x) const
  {
    return x < min_ ? min_ : (x > max_ ? max_ : x);
  }
  Scalar min_, max_;
};

}  // namespace grid_map
#endif  // GRID_MAP_CORE__EIGEN_PLUGINS__FUNCTORS_HPP_
