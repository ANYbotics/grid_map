/*
 * CostmapConverter.hpp
 *
 *  Created on: Nov 23, 2016
 *      Author: Peter Fankhauser, ETH Zurich
 *              Stefan Kohlbrecher, TU Darmstadt
 */

#pragma once

// ROS
#include <costmap_2d/costmap_2d.h>
#include <ros/ros.h>

// STD
#include <vector>

namespace grid_map {

template<typename MapType>
class CostmapConverter
{
 public:
  CostmapConverter()
  {
    initializeConversionTable();
  }

  virtual ~CostmapConverter()
  {
  }

  void initializeFromCostmap2d(const costmap_2d::Costmap2D& costmap2d, MapType& outputMap)
  {
    Length length(costmap2d.getSizeInMetersX(), costmap2d.getSizeInMetersY());
    // Different conventions: Costmap2d returns the *centerpoint* of the last cell in the map.
    length += Length::Constant(0.5 * costmap2d.getResolution());
    const double resolution =  costmap2d.getResolution();
    Position position(costmap2d.getOriginX(), costmap2d.getOriginY());
    // Different conventions
    position += Position(0.5 * length);
    outputMap.setGeometry(length, resolution, position);
  }

  bool fromCostmap2d(const costmap_2d::Costmap2D& costmap2d, const std::string& layer,
                     MapType& outputMap)
  {
    // Check compliance.
    Size size(costmap2d.getSizeInCellsX(), costmap2d.getSizeInCellsY());
    if ((outputMap.getSize() != size).any()) {
      ROS_ERROR("Costmap2D and output map have different sizes!");
      return false;
    }
    if (!outputMap.getStartIndex().isZero()) {
      ROS_ERROR("CostmapConverter::fromCostmap2d() does not support non-zero start indices!");
      return false;
    }

    // Copy data.
    // Reverse iteration is required because of different conventions
    // between Costmap2d and grid map.
    grid_map::Matrix data(size(0), size(1));
    const size_t nCells = outputMap.getSize().prod();
    for (size_t i = 0, j = nCells - 1; i < nCells; ++i, --j) {
      const unsigned char cost = costmap2d.getCharMap()[j];
      data(i) = (float) costTranslationTable_[cost];
    }

    outputMap.add(layer, data);
    return true;
  }

 private:
  void initializeConversionTable()
  {
    costTranslationTable_.resize(256);

    // Special values.
    costTranslationTable_[0] = 0; // NO obstacle
    costTranslationTable_[253] = 99; // INSCRIBED obstacle
    costTranslationTable_[254] = 100; // LETHAL obstacle
    costTranslationTable_[255] = -1; // UNKNOWN

    // Regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++) {
      costTranslationTable_[i] = char(1 + (97 * (i - 1)) / 251);
    }
  }

  std::vector<char> costTranslationTable_;
};

} /* namespace grid_map */
