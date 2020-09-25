/*
 * common.hpp
 *
 *  Created on: Nov 8, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <vector>
#include <random>


#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

namespace grid_map
{

class GridMapPclLoader;

namespace grid_map_pcl_test
{

using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

static const std::string layerName = "elevation";  // NOLINT
static const double savePointclouds = true;
extern std::mt19937 rndGenerator;

std::string getConfigFilePath();
std::string getTestDataFolderPath();
Pointcloud::Ptr concatenate(Pointcloud::Ptr cloud1, Pointcloud::Ptr cloud2);

std::vector<double> getNonNanElevationValues(const grid_map::GridMap & gridMap);
std::vector<Eigen::Vector3d> getNonNanElevationValuesWithCoordinates(
  const grid_map::GridMap & gridMap);

Pointcloud::Ptr createNormallyDistributedBlobOfPoints(
  unsigned int nPoints, double mean,
  double stdDev, std::mt19937 * generator);
Pointcloud::Ptr createNoisyPlanePointcloud(
  unsigned int nPoints, double minXY, double maxXY,
  double meanZ, double stdDevZ, std::mt19937 * generator);
Pointcloud::Ptr createPerfectPlane(
  unsigned int nPoints, double minXY, double maxXY,
  double desiredHeight, std::mt19937 * generator);

void runGridMapPclLoaderOnInputCloud(
  Pointcloud::ConstPtr inputCloud,
  grid_map::GridMapPclLoader * gridMapPclLoader);
void runGridMapPclLoaderOnInputCloudAndSavePointCloud(
  Pointcloud::ConstPtr inputCloud,
  grid_map::GridMapPclLoader * gridMapPclLoader,
  const std::string & filename);

Pointcloud::Ptr createStepTerrain(
  unsigned int nPoints, double minXY, double maxXY, double zHigh,
  double zLow, double stdDevZ, std::mt19937 * generator,
  double * center);

std::string getTestPcdFilePath();

}  // namespace grid_map_pcl_test
}  // namespace grid_map
#endif  // TEST_HELPERS_HPP_
