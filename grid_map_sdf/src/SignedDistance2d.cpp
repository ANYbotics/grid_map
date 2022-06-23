/*
* SignedDistance2d.cpp
*
*  Created on: Jul 10, 2020
*      Author: Ruben Grandia
*   Institute: ETH Zurich
*/

#include "grid_map_sdf/SignedDistance2d.hpp"

#include "grid_map_sdf/PixelBorderDistance.hpp"

namespace grid_map {
namespace signed_distance_field {

namespace internal {
struct DistanceLowerBound {
 float v;      // origin of bounding function
 float f;      // functional offset at the origin
 float z_lhs;  // lhs of interval where this bound holds
 float z_rhs;  // rhs of interval where this lower bound holds
};

/**
* 1D Euclidean Distance Transform based on: http://cs.brown.edu/people/pfelzens/dt/
* Adapted to work on Eigen objects directly
* Optimized computation of s.
* Some optimization to not keep track of bounds that lie fully outside the grid.
*/
std::vector<DistanceLowerBound>::iterator fillLowerBounds(const Eigen::Ref<Eigen::VectorXf>& squareDistance1d,
                                                         std::vector<DistanceLowerBound>& lowerBounds, Eigen::Index start) {
 const auto n = squareDistance1d.size();
 const auto nFloat = static_cast<float>(n);
 const auto startFloat = static_cast<float>(start);

 // Initialize
 auto rhsBoundIt = lowerBounds.begin();
 *rhsBoundIt = DistanceLowerBound{startFloat, squareDistance1d[start], -INF, INF};
 auto firstBoundIt = lowerBounds.begin();

 // Compute bounds to the right of minimum
 float qFloat = rhsBoundIt->v + 1.0F;
 for (Eigen::Index q = start + 1; q < n; ++q) {
   // Storing this by value gives better performance (removed indirection?)
   const float fq = squareDistance1d[q];

   float s = equidistancePoint(qFloat, fq, rhsBoundIt->v, rhsBoundIt->f);
   if (s < nFloat) {  // Can ignore the lower bound derived from this point if it is only active outsize of [start, n]
     // Search backwards in bounds until s is within [z_lhs, z_rhs]
     while (s < rhsBoundIt->z_lhs) {
       --rhsBoundIt;
       s = equidistancePoint(qFloat, fq, rhsBoundIt->v, rhsBoundIt->f);
     }
     if (s > startFloat) {     // Intersection is within [start, n]. Adjust current lowerbound and insert the new one after
       rhsBoundIt->z_rhs = s;  // Update the bound that comes before
       ++rhsBoundIt;           // insert new bound after.
       *rhsBoundIt = DistanceLowerBound{qFloat, fq, s, INF};  // Valid from s till infinity
     } else {  // Intersection is outside [0, n]. This means that the new bound dominates all previous bounds
       *rhsBoundIt = DistanceLowerBound{qFloat, fq, -INF, INF};
       firstBoundIt = rhsBoundIt;  // No need to keep other bounds, so update the first bound iterator to this one.
     }
   }

   // Increment to follow loop counter as float
   qFloat += 1.0F;
 }

 return firstBoundIt;
}

void extractSquareDistances(Eigen::Ref<Eigen::VectorXf> squareDistance1d, std::vector<DistanceLowerBound>::const_iterator lowerBoundIt,
                           Eigen::Index start) {
 const auto n = squareDistance1d.size();

 // Store active bound by value to remove indirection
 auto lastz = lowerBoundIt->z_rhs;

 auto qFloat = static_cast<float>(start);
 for (Eigen::Index q = start; q < n; ++q) {
   if (squareDistance1d[q] > 0.0F) {  // Observe that if squareDistance1d[q] == 0.0, this is already the minimum and it will stay 0.0
     // Find the new active lower bound if q no longer belongs to current interval
     if (qFloat > lastz) {
       do {
         ++lowerBoundIt;
       } while (lowerBoundIt->z_rhs < qFloat);
       lastz = lowerBoundIt->z_rhs;
     }

     squareDistance1d[q] = squarePixelBorderDistance(qFloat, lowerBoundIt->v, lowerBoundIt->f);
   }

   qFloat += 1.0F;
 }
}

/**
* Same as extractSquareDistances, but takes the sqrt as a final step.
* Because several cells will have a value of 0.0 (obstacle / free space label), we can skip the sqrt computation for those.
*/
void extractDistances(Eigen::Ref<Eigen::VectorXf> squareDistance1d,
                     std::vector<DistanceLowerBound>::const_iterator lowerBoundIt, Eigen::Index start) {
 const auto n = squareDistance1d.size();

 // Store active bound by value to remove indirection
 auto lastz = lowerBoundIt->z_rhs;

 auto qFloat = static_cast<float>(start);
 for (Eigen::Index q = start; q < n; ++q) {
   if (squareDistance1d[q] > 0.0F) {  // Observe that if squareDistance1d[q] == 0.0, this is already the minimum and it will stay 0.0
     // Find the new active lower bound if q no longer belongs to current interval
     if (qFloat > lastz) {
       do {
         ++lowerBoundIt;
       } while (lowerBoundIt->z_rhs < qFloat);
       lastz = lowerBoundIt->z_rhs;
     }

     squareDistance1d[q] = std::sqrt(squarePixelBorderDistance(qFloat, lowerBoundIt->v, lowerBoundIt->f));
   }

   qFloat += 1.0F;
 }
}

/**
* Find the location of the last zero value from the front
*/
Eigen::Index lastZeroFromFront(const Eigen::Ref<Eigen::VectorXf>& squareDistance1d) {
 const auto n = squareDistance1d.size();

 for (Eigen::Index q = 0; q < n; ++q) {
   if (squareDistance1d[q] > 0.0F) {
     if (q > 0) {
       return q - 1;
     } else {
       return 0;
     }
   }
 }
 return n;
}

inline void squaredDistanceTransform_1d_inplace(Eigen::Ref<Eigen::VectorXf> squareDistance1d,
                                               std::vector<DistanceLowerBound>& lowerBounds) {
 auto start = lastZeroFromFront(squareDistance1d);

 // Only need to process line if there are nonzero elements. Also the first zeros stay untouched.
 if (start < squareDistance1d.size()) {
   auto startIt = fillLowerBounds(squareDistance1d, lowerBounds, start);
   extractSquareDistances(squareDistance1d, startIt, start);
 }
}

/**
* Same as above, but takes sqrt as final step (within the same loop)
* @param squareDistance1d : input as squared distance, output is the distance after sqrt.
* @param lowerBounds : work vector
*/
inline void distanceTransform_1d_inplace(Eigen::Ref<Eigen::VectorXf> squareDistance1d, std::vector<DistanceLowerBound>& lowerBounds) {
 auto start = lastZeroFromFront(squareDistance1d);

 // Only need to process line if there are nonzero elements. Also the first zeros stay untouched.
 if (start < squareDistance1d.size()) {
   auto startIt = fillLowerBounds(squareDistance1d, lowerBounds, start);
   extractDistances(squareDistance1d, startIt, start);
 }
}

void computePixelDistance2dTranspose(Matrix& input, Matrix& distanceTranspose) {
 const auto n = input.rows();
 const auto m = input.cols();

 // Allocate a buffer big enough for processing both rowise and columnwise
 std::vector<DistanceLowerBound> lowerBounds(std::max(n, m));

 // Process columns
 for (Eigen::Index i = 0; i < m; ++i) {
   squaredDistanceTransform_1d_inplace(input.col(i), lowerBounds);
 }

 // Process rows (= columns after transpose).
 distanceTranspose = input.transpose();
 for (Eigen::Index i = 0; i < n; ++i) {
   // Fuses square distance algorithm and taking sqrt.
   distanceTransform_1d_inplace(distanceTranspose.col(i), lowerBounds);
 }
}

// Initialize with square distance in height direction in pixel units if above the surface
void initializeObstacleDistance(const Matrix& elevationMap, Matrix& result, float height, float resolution) {
 /* Vectorized implementation of:
  * if (height > elevation) {
  *    const auto diff = (height - elevation) / resolution;
  *    return diff * diff;
  * } else {
  *    return 0.0F;
  * }
  */
 result = ((1.0F / resolution) * (height - elevationMap.array()).cwiseMax(0.0F)).square();
}

// Initialize with square distance in height direction in pixel units if below the surface
void initializeObstacleFreeDistance(const Matrix& elevationMap, Matrix& result, float height, float resolution) {
 /* Vectorized implementation of:
  * if (height < elevation) {
  *    const auto diff = (height - elevation) / resolution;
  *    return diff * diff;
  * } else {
  *    return 0.0F;
  * }
  */
 result = ((1.0F / resolution) * (height - elevationMap.array()).cwiseMin(0.0F)).square();
}

void pixelDistanceToFreeSpaceTranspose(const Matrix& elevationMap, Matrix& sdfObstacleFree, Matrix& tmp, float height, float resolution) {
 internal::initializeObstacleFreeDistance(elevationMap, tmp, height, resolution);
 internal::computePixelDistance2dTranspose(tmp, sdfObstacleFree);
}

void pixelDistanceToObstacleTranspose(const Matrix& elevationMap, Matrix& sdfObstacleTranspose, Matrix& tmp, float height,
                                     float resolution) {
 internal::initializeObstacleDistance(elevationMap, tmp, height, resolution);
 internal::computePixelDistance2dTranspose(tmp, sdfObstacleTranspose);
}

Matrix signedDistanceFromOccupancyTranspose(const Eigen::Matrix<bool, -1, -1>& occupancyGrid, float resolution) {
 // Compute pixel distance to obstacles
 Matrix sdfObstacle;
 Matrix init = occupancyGrid.unaryExpr([=](bool val) { return (val) ? 0.0F : INF; });
 internal::computePixelDistance2dTranspose(init, sdfObstacle);

 // Compute pixel distance to obstacle free space
 Matrix sdfObstacleFree;
 init = occupancyGrid.unaryExpr([=](bool val) { return (val) ? INF : 0.0F; });
 internal::computePixelDistance2dTranspose(init, sdfObstacleFree);

 return resolution * (sdfObstacle - sdfObstacleFree);
}

}  // namespace internal

void signedDistanceAtHeightTranspose(const Matrix& elevationMap, Matrix& sdfTranspose, Matrix& tmp, Matrix& tmpTranspose, float height,
                                    float resolution, float minHeight, float maxHeight) {
 const bool allPixelsAreObstacles = height < minHeight;
 const bool allPixelsAreFreeSpace = height > maxHeight;

 if (allPixelsAreObstacles) {
   internal::pixelDistanceToFreeSpaceTranspose(elevationMap, sdfTranspose, tmp, height, resolution);

   sdfTranspose *= -resolution;
 } else if (allPixelsAreFreeSpace) {
   internal::pixelDistanceToObstacleTranspose(elevationMap, sdfTranspose, tmp, height, resolution);

   sdfTranspose *= resolution;
 } else {  // This layer contains a mix of obstacles and free space
   internal::pixelDistanceToObstacleTranspose(elevationMap, sdfTranspose, tmp, height, resolution);
   internal::pixelDistanceToFreeSpaceTranspose(elevationMap, tmpTranspose, tmp, height, resolution);

   sdfTranspose = resolution * (sdfTranspose - tmpTranspose);
 }
}

Matrix signedDistanceAtHeight(const Matrix& elevationMap, float height, float resolution, float minHeight, float maxHeight) {
 Matrix sdfTranspose;
 Matrix tmp;
 Matrix tmpTranspose;

 signedDistanceAtHeightTranspose(elevationMap, sdfTranspose, tmp, tmpTranspose, height, resolution, minHeight, maxHeight);
 return sdfTranspose.transpose();
}

Matrix signedDistanceFromOccupancy(const Eigen::Matrix<bool, -1, -1>& occupancyGrid, float resolution) {
 auto obstacleCount = occupancyGrid.count();
 bool hasObstacles = obstacleCount > 0;
 if (hasObstacles) {
   bool hasFreeSpace = obstacleCount < occupancyGrid.size();
   if (hasFreeSpace) {
     return internal::signedDistanceFromOccupancyTranspose(occupancyGrid, resolution).transpose();
   } else {
     // Only obstacles -> distance is minus infinity everywhere
     return Matrix::Constant(occupancyGrid.rows(), occupancyGrid.cols(), -INF);
   }
 } else {
   // No obstacles -> planar distance is infinite
   return Matrix::Constant(occupancyGrid.rows(), occupancyGrid.cols(), INF);
 }
}

}  // namespace signed_distance_field
}  // namespace grid_map