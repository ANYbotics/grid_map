/*
 * GridMapMsgHelpers.hpp
 *
 *  Created on: Sep 8, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMapMsgHelpers.hpp"

// Boost
#include <boost/assign.hpp>

namespace grid_map {

const int nDimensions()
{
  return 2;
}

std::map<StorageIndices, std::string> storageIndexNames = boost::assign::map_list_of
    (StorageIndices::Column,  "column_index")
    (StorageIndices::Row, "row_index");

bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3i& colorVector)
{
  colorVector(0) = (colorValue >> 16) & 0x0000ff;
  colorVector(1) = (colorValue >> 8) & 0x0000ff;
  colorVector(2) =  colorValue & 0x0000ff;
  return true;
}

bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3f& colorVector)
{
  Eigen::Vector3i tempColorVector;
  colorValueToVector(colorValue, tempColorVector);
  colorVector = ((tempColorVector.cast<float>()).array() / 255.0).matrix();
  return true;
}

bool colorVectorToValue(const Eigen::Vector3i& colorVector, unsigned long& colorValue)
{
  colorValue = ((int)colorVector(0)) << 16 | ((int)colorVector(1)) << 8 | ((int)colorVector(2));
  return true;
}

void colorVectorToValue(const Eigen::Vector3i& colorVector, float& colorValue)
{
  int color = (colorVector(0) << 16) + (colorVector(1) << 8) + colorVector(2);
  colorValue = *reinterpret_cast<float*>(&color);
}

} /* namespace */
