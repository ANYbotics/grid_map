/*
 * PointcloudCreator.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "PointcloudCreator.hpp"

namespace grid_map
{
namespace grid_map_pcl_test
{

Pointcloud::Ptr PointcloudCreator::createNoisyPointcloudOfStepTerrain(
  double * stepLocationX,
  double * zHigh, double * zLow,
  double * stdDev)
{
  std::uniform_real_distribution<double> heightDist(-2.0, 2.0);
  const double maxXY = 3.0;
  const double minXY = -3.0;
  *stdDev = 0.01;
  const unsigned int nPointsInCloud = 1000000;
  *zHigh = heightDist(rndGenerator) + 2.1;
  *zLow = heightDist(rndGenerator) - 2.1;
  auto cloud = grid_map_pcl_test::createStepTerrain(
    nPointsInCloud, minXY, maxXY, *zHigh, *zLow,
    *stdDev, &rndGenerator, stepLocationX);

  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createBlobOfPoints(double * mean, double * stdDev)
{
  const unsigned int nPointsInCloud = 10000;
  std::uniform_real_distribution<double> meanDist(-10.0, 10.0);
  std::uniform_real_distribution<double> sigmaDist(0.001, 0.1);

  *mean = meanDist(rndGenerator);
  *stdDev = sigmaDist(rndGenerator);

  auto cloud = grid_map_pcl_test::createNormallyDistributedBlobOfPoints(
    nPointsInCloud, *mean,
    *stdDev, &rndGenerator);

  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createVerticesOfASquare(double * x, double * y)
{
  grid_map_pcl_test::Pointcloud::Ptr cloud(new grid_map_pcl_test::Pointcloud());
  std::uniform_real_distribution<double> zDist(-10.0, 10.0);
  std::uniform_int_distribution<int> xDist(10, 20);
  std::uniform_int_distribution<int> yDist(25, 40);
  *x = xDist(rndGenerator);
  *y = yDist(rndGenerator);

  cloud->points.push_back(grid_map_pcl_test::Point(*x, 0.0, zDist(rndGenerator)));
  cloud->points.push_back(grid_map_pcl_test::Point(-(*x), 0.0, zDist(rndGenerator)));
  cloud->points.push_back(grid_map_pcl_test::Point(0.0, *y, zDist(rndGenerator)));
  cloud->points.push_back(grid_map_pcl_test::Point(0.0, -(*y), zDist(rndGenerator)));
  cloud->is_dense = true;

  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createNoisyDoublePlane(double * minZ, double * stdDevZ)
{
  std::uniform_real_distribution<double> upperPlaneZDist(0.0, 10.0);
  std::uniform_real_distribution<double> lowerPlaneZDist(-10.0, -5.0);
  std::uniform_real_distribution<double> stdDevDist(0.001, 0.1);

  const double maxXY = 1.0;
  const double minXY = -1.0;
  *stdDevZ = stdDevDist(rndGenerator);

  // make it very dense such that we ensure that there is enough points
  // that will make it into the cell
  // todo should sample in a better way
  const unsigned int nPointsInCloud = 1000000;
  *minZ = lowerPlaneZDist(rndGenerator);
  auto cloudLower = grid_map_pcl_test::createNoisyPlanePointcloud(
    nPointsInCloud, minXY, maxXY,
    *minZ, *stdDevZ, &rndGenerator);
  auto cloudUpper = grid_map_pcl_test::createNoisyPlanePointcloud(
    nPointsInCloud, minXY, maxXY,
    upperPlaneZDist(rndGenerator),
    *stdDevZ, &rndGenerator);
  auto cloud = grid_map_pcl_test::concatenate(cloudUpper, cloudLower);

  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createNoisyPlane(double * height, double * stdDevZ)
{
  std::uniform_real_distribution<double> heightDist(-10.0, 10.0);
  std::uniform_real_distribution<double> stdDevDist(0.001, 0.1);

  const double maxXY = 1.0;
  const double minXY = -1.0;
  *stdDevZ = stdDevDist(rndGenerator);

  // make it very dense such that we ensure that there is enough points
  // that will make it into the cell
  // todo should sample in a better way
  const unsigned int nPointsInCloud = 1000000;
  *height = heightDist(rndGenerator);

  auto cloud = grid_map_pcl_test::createNoisyPlanePointcloud(
    nPointsInCloud, minXY, maxXY, *height,
    *stdDevZ, &rndGenerator);

  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createPerfectPlane(double * height)
{
  std::uniform_real_distribution<double> heightDist(-10.0, 10.0);
  const double maxXY = 3.0;
  const double minXY = -3.0;
  const unsigned int nPointsInCloud = 100000;
  *height = heightDist(rndGenerator);
  auto cloud = grid_map_pcl_test::createPerfectPlane(
    nPointsInCloud, minXY, maxXY, *height,
    &rndGenerator);

  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createNBlobsAboveEachOther(
  double * minZ, double * stdDevZ,
  int * nBlobs)
{
  const unsigned int nPointsInCloud = 1000;
  std::uniform_real_distribution<double> sigmaDist(0.001, 0.015);
  std::uniform_real_distribution<double> minZDist(-10.0, 10.0);
  std::uniform_int_distribution<int> nDist(10, 20);
  const double zStep = 2.0;

  *nBlobs = nDist(rndGenerator);
  *minZ = minZDist(rndGenerator);
  *stdDevZ = sigmaDist(rndGenerator);

  Pointcloud::Ptr cloud(new Pointcloud());

  for (int i = 0; i < *nBlobs; ++i) {
    auto blob = grid_map_pcl_test::createNormallyDistributedBlobOfPoints(
      nPointsInCloud,
      *minZ + i * zStep,
      *stdDevZ, &rndGenerator);
    auto temp = concatenate(cloud, blob);
    cloud = temp;
  }

  return cloud;
}

}  // namespace grid_map_pcl_test
}  // namespace grid_map
