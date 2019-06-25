/*
 * CostmapConverter.hpp
 *
 *  Created on: Nov 23, 2016
 *      Author: Peter Fankhauser, ETH Zurich
 *              Stefan Kohlbrecher, TU Darmstadt
 *              Daniel Stonier, Yujin Robot
 */

#pragma once

#include <grid_map_core/grid_map_core.hpp>

// ROS
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tf.h>

// STD
#include <vector>

namespace grid_map {

/*!
 * @brief Direct cost translations for costmap_2d -> grid/cost maps.
 *
 * This maps the cost directly, simply casting from the underlying
 * unsigned char to whatever onto float fields between 0.0 and 100.0 with
 * reserved values allocated for the costmap_2d.
 */
class Costmap2DDirectTranslationTable
{
 public:
  Costmap2DDirectTranslationTable() {}

  template<typename DataType>
  static void create(std::vector<DataType>& costTranslationTable) {
    costTranslationTable.resize(256);
    for (unsigned int i = 0; i < costTranslationTable.size(); ++i ) {
      costTranslationTable[i] = static_cast<DataType>(i);
    }
  }
};

/**
 * @brief Cost translations to [0, 100] for costmap_2d -> grid/cost maps.
 *
 * This maps the cost onto float fields between 0.0 and 100.0 with
 * reserved values allocated for the costmap_2d.
 */
class Costmap2DCenturyTranslationTable
{
 public:
  Costmap2DCenturyTranslationTable() {}

  template<typename DataType>
  static void create(std::vector<DataType>& costTranslationTable) {
    costTranslationTable.resize(256);
    costTranslationTable[0] = 0.0;     // costmap_2d::FREE_SPACE;
    costTranslationTable[253] = 99.0;  // costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    costTranslationTable[254] = 100.0; // costmap_2d::LETHAL_OBSTACLE
    costTranslationTable[255] = -1.0;  // costmap_2d::NO_INFORMATION

    // Regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++) {
      costTranslationTable[i] = char(1 + (97 * (i - 1)) / 251);
    }
  }
};

template<typename DataType>
class Costmap2DDefaultTranslationTable : public Costmap2DDirectTranslationTable {};

template<>
class Costmap2DDefaultTranslationTable<float> : public Costmap2DCenturyTranslationTable {};

/*****************************************************************************
** Converter
*****************************************************************************/

/**
 * @brief Convert Costmap2DRos objects into cost or grid maps.
 *
 * @tparam MapType : either grid_map::GridMap or cost_map::CostMap
 * @tparam TranslationTable : class that creates a cost -> new type conversion table
 *
 * @sa Costmap2DDirectTranslationTable, Costmap2DCenturyTranslationTable
 */
template<typename MapType, typename TranslationTable=Costmap2DDefaultTranslationTable<typename MapType::DataType>>
class Costmap2DConverter
{
public:

  /*!
   * Initializes the cost translation table with the default policy for the template
   * map type class.
   */
  Costmap2DConverter()
  {
    TranslationTable::create(costTranslationTable_);
  }

  virtual ~Costmap2DConverter()
  {
  }

  void initializeFromCostmap2D(costmap_2d::Costmap2DROS& costmap2d, MapType& outputMap)
  {
    initializeFromCostmap2D(*(costmap2d.getCostmap()), outputMap);
    outputMap.setFrameId(costmap2d.getGlobalFrameID());
  }

  void initializeFromCostmap2D(const costmap_2d::Costmap2D& costmap2d, MapType& outputMap)
  {
    const double resolution = costmap2d.getResolution();
    Length length(costmap2d.getSizeInCellsX()*resolution, costmap2d.getSizeInCellsY()*resolution);
    Position position(costmap2d.getOriginX(), costmap2d.getOriginY());
    // Different conventions.
    position += Position(0.5 * length);
    outputMap.setGeometry(length, resolution, position);
  }

  /*!
   * Load the costmap2d into the specified layer.
   * @warning This does not lock the costmap2d object, you should take care to do so from outside this function.
   * @param costmap2d from this costmap type object.
   * @param layer the name of the layer to insert.
   * @param outputMap to this costmap type object.
   * @return true if successful, false otherwise.
   */
  bool addLayerFromCostmap2D(const costmap_2d::Costmap2D& costmap2d,
                             const std::string& layer,
                             MapType& outputMap)
  {
    // Check compliance.
    Size size(costmap2d.getSizeInCellsX(), costmap2d.getSizeInCellsY());
    if ((outputMap.getSize() != size).any()) {
      errorMessage_ = "Costmap2D and output map have different sizes!";
      return false;
    }
    if (!outputMap.getStartIndex().isZero()) {
      errorMessage_ = "Does not support non-zero start indices!";
      return false;
    }
    // Copy data.
    // Reverse iteration is required because of different conventions
    // between Costmap2D and grid map.
    typename MapType::Matrix data(size(0), size(1));
    const size_t nCells = outputMap.getSize().prod();
    for (size_t i = 0, j = nCells - 1; i < nCells; ++i, --j) {
      const unsigned char cost = costmap2d.getCharMap()[j];
      data(i) = (float) costTranslationTable_[cost];
    }

    outputMap.add(layer, data);
    return true;
  }

  /*!
   * Load the costmap2d into the specified layer.
   * @warning This does not lock the costmap2d object, you should take care to do so from outside this function.
   * @param costmap2d from this costmap type object.
   * @param layer the name of the layer to insert.
   * @param outputMap to this costmap type object.
   * @return true if successful, false otherwise.
   */
  bool addLayerFromCostmap2D(costmap_2d::Costmap2DROS& costmap2d, const std::string& layer,
                             MapType& outputMap)
  {
    return addLayerFromCostmap2D(*(costmap2d.getCostmap()), layer, outputMap);
  }

  /*!
   * Initialize the output map at the robot pose of the underlying Costmap2DROS object.
   *
   * Note: This needs some beyond just fixing the center of the output map to the robot pose
   * itself. To avoid introducing error, you want to shift the center so the
   * underlying costmap2d cells align with the newly created map object. This does
   * that here.
   *
   * @warning this does not lock the costmap2d object, you should take care to do so from
   * outside this function in scope with any addLayerFromCostmap2DAtRobotPose calls you wish to make.
   *
   * @param costmap2d : the underlying Costmap2DROS object
   * @param geometry: size of the subwindow to snapshot around the robot pose
   * @param outputMap : initialise the output map with the required parameters
   */
  bool initializeFromCostmap2DAtRobotPose(costmap_2d::Costmap2DROS& costmap2d, const Length& length,
                                          MapType& outputMap)
  {
    const double resolution = costmap2d.getCostmap()->getResolution();

    // Get the Robot Pose Transform.
#if ROS_VERSION_MINIMUM(1,14,0)
    geometry_msgs::PoseStamped tfPose;
    if(!costmap2d.getRobotPose(tfPose))
    {
      errorMessage_ =  "Could not get robot pose, is it actually published?";
      return false;
    }
    Position robotPosition(tfPose.pose.position.x, tfPose.pose.position.y);
#else
    tf::Stamped<tf::Pose> tfPose;
    if(!costmap2d.getRobotPose(tfPose))
    {
      errorMessage_ =  "Could not get robot pose, is it actually published?";
      return false;
    }
    Position robotPosition(tfPose.getOrigin().x() , tfPose.getOrigin().y());
#endif
    // Determine new costmap origin.
    Position rosMapOrigin(costmap2d.getCostmap()->getOriginX(), costmap2d.getCostmap()->getOriginY());
    Position newCostMapOrigin;

    // Note:
    //   You cannot directly use the robot pose as the new 'costmap center'
    //   since the underlying grid is not necessarily exactly aligned with
    //   that (two cases to consider, rolling window and globally fixed).
    //
    // Relevant diagrams:
    //  - https://github.com/anybotics/grid_map

    // Float versions of the cell co-ordinates, use static_cast<int> to get the indices.
    Position robotCellPosition = (robotPosition - rosMapOrigin) / resolution;

    // if there is an odd number of cells
    //   centre of the new grid map in the centre of the current cell
    // if there is an even number of cells
    //   centre of the new grid map at the closest vertex between cells
    // of the current cell
    int numberOfCellsX = length.x()/resolution;
    int numberOfCellsY = length.y()/resolution;
    if (numberOfCellsX % 2) {  // odd
      newCostMapOrigin(0) = std::floor(robotCellPosition.x())*resolution + resolution/2.0 + rosMapOrigin.x();
    } else {
      newCostMapOrigin(0) = std::round(robotCellPosition.x())*resolution + rosMapOrigin.x();
    }
    if (numberOfCellsY % 2) {  // odd
      newCostMapOrigin(1) = std::floor(robotCellPosition.y())*resolution + resolution/2.0 + rosMapOrigin.y();
    } else {
      newCostMapOrigin(1) = std::round(robotCellPosition.y())*resolution + rosMapOrigin.y();
    }

    // TODO check the robot pose is in the window
    // TODO check the geometry fits within the costmap2d window

    // Initialize the output map.
    outputMap.setFrameId(costmap2d.getGlobalFrameID());
    outputMap.setTimestamp(ros::Time::now().toNSec());
    outputMap.setGeometry(length, resolution, newCostMapOrigin);
    return true;
  }

  /**
   * Fill a layer in the output map with data from the subwindow centered at the robot pose.
   * @warning This does not lock the costmap2d object, you should take care to do so from
   * outside this function in scope with any initializeFromCostmap2DAtRobotPose calls
   * you wish to make.
   * @param costmap2d the underlying Costmap2DROS object.
   * @param layer the layer name to add.
   * @param outputMap the initialized and filled in map output map.
   */
  bool addLayerFromCostmap2DAtRobotPose(costmap_2d::Costmap2DROS& costmap2d,
                                        const std::string& layer, MapType& outputMap)
  {
    /****************************************
    ** Asserts
    ****************************************/
    if (outputMap.getResolution() != costmap2d.getCostmap()->getResolution()) {
      errorMessage_ = "Costmap2D and output map have different resolutions!";
      return false;
    }
    // 1) would be nice to check the output map centre has been initialised where it should be
    //      i.e. the robot pose didn't move since or the initializeFrom wasn't called
    //    but this would mean listening to tf's again and anyway, it gets shifted to make sure
    //    the costmaps align, so these wouldn't be exactly the same anyway
    // 2) check the geometry fits inside the costmap2d subwindow is done below

    // Properties.
    const double resolution = costmap2d.getCostmap()->getResolution();
    const Length geometry = outputMap.getLength();
    const Position position = outputMap.getPosition();

    // Copy data.
    bool isValidWindow = false;
    costmap_2d::Costmap2D costmapSubwindow;
    // TODO
    isValidWindow = costmapSubwindow.copyCostmapWindow(
                            *(costmap2d.getCostmap()),
                            position.x() - geometry.x() / 2.0, // subwindow_bottom_left_x
                            position.y() - geometry.y() / 2.0, // subwindow_bottom_left_y
                            geometry.x(),
                            geometry.y());
    if (!isValidWindow) {
      // handle differently - e.g. copy the internal part only and lethal elsewhere, but other parts would have to handle being outside too
      errorMessage_ = "Subwindow landed outside the costmap, aborting.";
      return false;
    }
    addLayerFromCostmap2D(costmapSubwindow, layer, outputMap);
    return true;
  }

  /**
   * @brief Human readable error message string.
   *
   * Can be used to get a human readable description of the error type after
   * one of the class methods has failed.
   *
   * @return std::string
   */
  std::string errorMessage() const { return errorMessage_; }

private:
  std::vector<typename MapType::DataType> costTranslationTable_;
  std::string errorMessage_;
};

} // namespace grid_map
