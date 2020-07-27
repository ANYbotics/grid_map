/*
 * GridMapMsgHelpers.hpp
 *
 *  Created on: Sep 8, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

// Boost
#include <boost/assign.hpp>

#include <map>
#include <string>

#include "grid_map_ros/GridMapMsgHelpers.hpp"

namespace grid_map
{

int nDimensions()
{
  return 2;
}

std::map<StorageIndices, std::string> storageIndexNames = boost::assign::map_list_of(
  StorageIndices::Column, "column_index")(StorageIndices::Row, "row_index");

}  // namespace grid_map
