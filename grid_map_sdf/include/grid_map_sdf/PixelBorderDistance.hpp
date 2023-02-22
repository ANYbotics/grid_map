/*
 * PixelBorderDistance.h
 *
 *  Created on: Aug 7, 2020
 *      Author: Ruben Grandia
 *   Institute: ETH Zurich
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

namespace grid_map {
namespace signed_distance_field {

/**
 * Returns distance between the center of a pixel and the border of an other pixel.
 * Returns zero if the center is inside the other pixel.
 * Pixels are assumed to have size 1.0F
 * @param i : location of pixel 1
 * @param j : location of pixel 2
 * @return : absolute distance between center of pixel 1 and the border of pixel 2
 */
inline float pixelBorderDistance(float i, float j) {
  return std::max(std::abs(i - j) - 0.5F, 0.0F);
}

/**
 * Returns square pixelBorderDistance, adding offset f.
 */
inline float squarePixelBorderDistance(float i, float j, float f) {
  const float d{pixelBorderDistance(i, j)};
  return d * d + f;
}

namespace internal {

/**
 * Return equidistancepoint between origin and pixel p (with p > 0) with offset fp
 */
inline float intersectionPointRightSideOfOrigin(float p, float fp) {
  /*
   * There are 5 different solutions
   * In decreasing order:
   * sol 1   in [p^2, inf]
   * sol 2   in [bound, p^2]
   * sol 3   in [-bound, bound]
   * sol 4   in [-p^2, -bound]
   * sol 5   in [-inf, -p^2]
   */
  const float pSquared{p * p};
  if (fp > pSquared) {
    return (pSquared + p + fp) / (2.0F * p);  // sol 1
  } else if (fp < -pSquared) {
    return (pSquared - p + fp) / (2.0F * p);  // sol 5
  } else {
    const float bound{pSquared - 2.0F * p + 1.0F};  // Always positive because (p > 0)
    if (fp > bound) {
      return 0.5F + std::sqrt(fp);  // sol 2
    } else if (fp < -bound) {
      return p - 0.5F - std::sqrt(-fp);  // sol 4
    } else {
      return (pSquared - p + fp) / (2.0F * p - 2.0F);  // sol 3
    }
  }
}

/**
 * Return equidistancepoint between origin and pixel p with offset fp
 */
inline float intersectionOffsetFromOrigin(float p, float fp) {
  if (p > 0.0F) {
    return intersectionPointRightSideOfOrigin(p, fp);
  } else {
    // call with negative p and flip the result
    return -intersectionPointRightSideOfOrigin(-p, fp);
  }
}

}  // namespace internal

/**
 * Return the point s in pixel space that is equally far from p and q (taking into account offsets fp, and fq)
 * It is the solution to the following equation:
 *      squarePixelBorderDistance(s, q, fq) == squarePixelBorderDistance(s, p, fp)
 */
inline float equidistancePoint(float q, float fq, float p, float fp) {
  assert(q == p ||
         std::abs(q - p) >= 1.0F);     // Check that q and p are proper pixel locations: either the same pixel or non-overlapping pixels
  assert((q == p) ? fp == fq : true);  // Check when q and p are equal, the offsets are also equal

  if (fp == fq) {  // quite common case when both pixels are of the same class (occupied / free)
    return 0.5F * (p + q);
  } else {
    float df{fp - fq};
    float dr{p - q};
    return internal::intersectionOffsetFromOrigin(dr, df) + q;
  }
}

}  // namespace signed_distance_field
}  // namespace grid_map