/*
 * PointcloudCreator.hpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "test_helpers.hpp"

namespace grid_map {
namespace grid_map_pcl_test {

/*
 * Creates point clouds with some noise in the z direction (which is
 * the direction perpendicular to the surface normal). This is an approximation
 * of a point cloud that would be obtained with a real sensor.
 */
class PointcloudCreator
{

 public:

  /*!
   * Creates a noisy point cloud of a step (i.e. two planes with some
   * displacement in z direction in between). Visualize stepTerrain.pcd inside
   * test/test_data folder for more info. Along the y dimension there is
   * step is located along the x dimension.
   * @param[out] x coordinate of the step
   * @param[out] height (z coordinate) of the higher plane
   * @param[out] height (z coordinate) of the lower plane
   * @param[out] std deviation in z direction of the points that form a plane
   * @return point cloud of the step terrain
   */
  static Pointcloud::Ptr createNoisyPointcloudOfStepTerrain(double *stepLocation, double *zHigh,
                                                            double *zLow, double *stdDev);

  /*!
   * Creates a blob of points (x,y,z). All coordinates (x,y,z)
   *  have the same mean and standard deviation. Visualize
   *   blob.pcd inside test/test_data folder for better understanding.
   * @param[out] mean coordinate of the blob of points
   * @param[out] standard deviation for points
   * @return point cloud that represents that blob of points
   */
  static Pointcloud::Ptr createBlobOfPoints(double *mean, double *stdDev);

  /*!
   * Creates a set of 4 points that form a square, with
   * the center in the origin of the coordinate system.
   * The points are (x,0,z), (-x,0,z), (0,y,z) and (0,-y,z).
   * Variables x,y,z are uniformly distributed. Visualize
   *   4pointSquare.pcd inside test/test_data folder for better understanding.
   * @param[out] variable x
   * @param[out] varialbe y
   * @return point cloud with 4 points that form a square
   */
  static Pointcloud::Ptr createVerticesOfASquare(double *x, double *y);

  /*!
   * Creates a plane where z coordinate of the points is normally distributed
   * with mean=height and standard deviation = stdDevZ (both height and
   * stdDevZ are uniformly distributed).
   * Location of x and y coordinate is an uniform distribution. Visualize
   *   noisyPlane.pcd inside test/test_data folder for better understanding.
   * @param[out] height of the points
   * @param[out] standard deviation in z coordinate
   * @return point cloud that represents a noisy plane
   */
  static Pointcloud::Ptr createNoisyPlane(double *height, double *stdDevZ);

  /*!
   * Creates two planes (top and bottom) where z coordinate of the
   * points is normally distributed with random mean and random
   * standard deviation. One plane is above the other. Location of
   * x and y coordinate is an uniform distribution. Visualize
   * doublePlane.pcd inside test/test_data folder for better understanding.
   * @param[out] height of the bottom plane
   * @param[out] standard deviation in z coordinate
   * @return point cloud that represents a two planes above each other
   */
  static Pointcloud::Ptr createNoisyDoublePlane(double *minZ, double *stdDevZ);

  /*!
   * Creates a plane with no noisy in z direction at a height that is
   * uniformly distributed. Both x and y coordinates of points
   * in the point cloud are uniformly distributed. Visualize
   * perfectPlane.pcd inside test/test_data folder for better understanding.
   * @param[out] height (z coordinate) of the plane
   * @return point cloud of a perfect plane
   */
  static Pointcloud::Ptr createPerfectPlane(double *height);

  /*!
   * Creates N blobs of points where N is uniformly distributed.
   *  All coordinates (x,y,z) standard deviation. Their mean is
   *  increased (deterministically) to ensure that blobs are apart from
   *  each other. Visualize Nblobs.pcd inside test/test_data folder
   *  for better understanding.
   * @param[out] height (z coordinate) of lowest blob
   * @param[out] stdDevZ of all the blobs
   * @param[out] number of blobs
   * @return point cloud with n blobs
   */
  static Pointcloud::Ptr createNBlobsAboveEachOther(double *minZ, double *stdDevZ, int *nBlobs);

};

} /* namespace grid_map_pcl_test*/
} /* namespace grid_map*/
