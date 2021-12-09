/*
 * common.cpp
 *
 *  Created on: Nov 8, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/package.h>

#include "grid_map_pcl/GridMapPclLoader.hpp"

#include "test_helpers.hpp"

namespace grid_map {
namespace grid_map_pcl_test {

std::mt19937 rndGenerator;

std::string getConfigFilePath() {
  std::string filename = getTestDataFolderPath() + "/parameters.yaml";
  return filename;
}

std::string getTestDataFolderPath() {
  std::string filename = ros::package::getPath("grid_map_pcl") + "/test/test_data";
  return filename;
}

std::vector<Eigen::Vector3d> getNonNanElevationValuesWithCoordinates(const grid_map::GridMap& gridMap) {
  std::vector<Eigen::Vector3d> nonNanCoordinates;
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    double value = gridMap.at(layerName, *iterator);
    if (!std::isnan(value)) {
      grid_map::Position position;
      gridMap.getPosition(grid_map::Index(*iterator), position);
      nonNanCoordinates.push_back(Eigen::Vector3d(position.x(), position.y(), value));
    }
  }

  return nonNanCoordinates;
}

std::vector<double> getNonNanElevationValues(const grid_map::GridMap& gridMap) {
  std::vector<double> nonNanElevations;
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    double value = gridMap.at(layerName, *iterator);
    if (!std::isnan(value)) {
      nonNanElevations.push_back(value);
    }
  }

  return nonNanElevations;
}

Pointcloud::Ptr createNormallyDistributedBlobOfPoints(unsigned int nPoints, double mean, double stdDev, std::mt19937* generator) {
  std::normal_distribution<double> normalDist(mean, stdDev);  // N

  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = normalDist(*generator);
    point.y = normalDist(*generator);
    point.z = normalDist(*generator);
    cloud->push_back(point);
  }

  return cloud;
}

Pointcloud::Ptr createNoisyPlanePointcloud(unsigned int nPoints, double minXY, double maxXY, double meanZ, double stdDevZ,
                                           std::mt19937* generator) {
  const double upperBound = maxXY;
  const double lowerBound = minXY;
  std::uniform_real_distribution<double> uniformDist(lowerBound, upperBound);
  const double mean = meanZ;
  const double stdDev = stdDevZ;
  std::normal_distribution<double> normalDist(mean, stdDev);  // N
  // std::cout << distr(generator) << '\n';

  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = uniformDist(*generator);
    point.y = uniformDist(*generator);
    point.z = normalDist(*generator);
    cloud->push_back(point);
  }

  return cloud;
}

Pointcloud::Ptr createPerfectPlane(unsigned int nPoints, double minXY, double maxXY, double desiredHeight, std::mt19937* generator) {
  const double upperBound = maxXY;
  const double lowerBound = minXY;
  std::uniform_real_distribution<double> uniformDist(lowerBound, upperBound);

  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = uniformDist(*generator);
    point.y = uniformDist(*generator);
    point.z = desiredHeight;
    cloud->push_back(point);
  }

  return cloud;
}

Pointcloud::Ptr concatenate(Pointcloud::Ptr cloud1, Pointcloud::Ptr cloud2) {
  // ghetto concatenate
  Pointcloud::Ptr concatenatedCloud(new grid_map_pcl_test::Pointcloud());
  concatenatedCloud->points.reserve(cloud1->points.size() + cloud2->points.size());

  std::copy(cloud2->points.begin(), cloud2->points.end(), std::back_inserter(concatenatedCloud->points));

  std::copy(cloud1->points.begin(), cloud1->points.end(), std::back_inserter(concatenatedCloud->points));

  return concatenatedCloud;
}

void runGridMapPclLoaderOnInputCloud(Pointcloud::ConstPtr inputCloud, grid_map::GridMapPclLoader* gridMapPclLoader) {
  gridMapPclLoader->loadParameters(getConfigFilePath());
  gridMapPclLoader->setInputCloud(inputCloud);
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  gridMapPclLoader->addLayerFromInputCloud(layerName);
}

void runGridMapPclLoaderOnInputCloudAndSavePointCloud(Pointcloud::ConstPtr inputCloud, grid_map::GridMapPclLoader* gridMapPclLoader,
                                                      const std::string& filename) {
  runGridMapPclLoaderOnInputCloud(inputCloud, gridMapPclLoader);
  gridMapPclLoader->savePointCloudAsPcdFile(filename);
}

Pointcloud::Ptr createStepTerrain(unsigned int nPoints, double minXY, double maxXY, double zHigh, double zLow, double stdDevZ,
                                  std::mt19937* generator, double* center) {
  *center = (maxXY + minXY) / 2.0;
  std::uniform_real_distribution<double> uniformDist(minXY, maxXY);
  std::normal_distribution<double> zLowDist(zLow, stdDevZ);
  std::normal_distribution<double> zHighDist(zHigh, stdDevZ);

  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = uniformDist(*generator);
    point.y = uniformDist(*generator);

    if (point.x > *center) {
      point.z = zHighDist(*generator);
    } else {
      point.z = zLowDist(*generator);
    }

    cloud->push_back(point);
  }

  return cloud;
}
void setVerbosityLevel(ros::console::levels::Level level) {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)) {
    ros::console::notifyLoggerLevelsChanged();
  }
}

std::string getTestPcdFilePath() {
  std::string filename = ros::package::getPath("grid_map_pcl") + "/test/test_data/plane_noisy.pcd";
  return filename;
}

} /* namespace grid_map_pcl_test */
} /* namespace grid_map*/
