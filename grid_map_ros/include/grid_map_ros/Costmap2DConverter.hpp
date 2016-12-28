/**
 * @file /cost_map_ros/include/cost_map_ros/costmap_2d_converter.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef grid_map_ros_COSTMAP_2D_CONVERTER_HPP_
#define grid_map_ros_COSTMAP_2D_CONVERTER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <sstream>
#include <tf/tf.h>
#include <vector>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace grid_map {

/*****************************************************************************
** Cost Translation Tables
*****************************************************************************/

/**
 * @brief Direct cost translations for costmap_2d -> cost/grid maps.
 *
 * This maps the cost directly, simply casting from the underlying
 * unsigned char to whatever onto float fields between 0.0 and 100.0 with
 * reserved values allocated for the costmap_2d
 */
class Costmap2DDirectTranslationTable {
public:
  Costmap2DDirectTranslationTable() {}

  template<typename DataType>
  void apply(std::vector<DataType>& cost_translation_table) const {
    cost_translation_table.resize(256);
    for (unsigned int i = 0; i < cost_translation_table.size(); ++i ) {
      cost_translation_table[i] = static_cast<DataType>(i);
    }
  }
};

/**
 * @brief Cost translations to [0, 100] for costmap_2d -> cost/grid maps
 *
 * This maps the cost onto float fields between 0.0 and 100.0 with
 * reserved values allocated for the costmap_2d
 */
class Costmap2DCenturyTranslationTable {
public:
  Costmap2DCenturyTranslationTable() {}

  void apply(std::vector<float>& cost_translation_table) const {
    cost_translation_table.resize(256);
    cost_translation_table[0] = 0.0;     // costmap_2d::FREE_SPACE;
    cost_translation_table[253] = 99.0;  // costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    cost_translation_table[254] = 100.0; // costmap_2d::LETHAL_OBSTACLE
    cost_translation_table[255] = -1.0;  // costmap_2d::NO_INFORMATION

    // Regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++) {
      cost_translation_table[i] = char(1 + (97 * (i - 1)) / 251);
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

template<typename MapType>
class Costmap2DConverter
{
public:
  /**
   * @brief Default constructor
   *
   * Initialises the cost translation table with the default policy for the template
   * map type class.
   **/
  Costmap2DConverter()
  {
    initializeCostTranslationTable(Costmap2DDefaultTranslationTable<typename MapType::DataType>());
  }

  /**
   * @brief Initialise the cost translation table with the user specified translation table functor.
   *
   * @param translation_table : user provided translation table
   */
  template <typename TranslationTable>
  Costmap2DConverter(const TranslationTable& translation_table)
  {
    initializeCostTranslationTable(translation_table);
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

  /**
   * @brief Load the costmap2d into the specified layer.
   *
   * @param costmap2d : from this costmap type object
   * @param layer : name of the layer to insert
   * @param outputMap : to this costmap type object
   *
   * @warning this does not lock the costmap2d object, you should take care to do so from outside this function.
   **/
  bool addLayerFromCostmap2D(const costmap_2d::Costmap2D& costmap2d,
                             const std::string& layer,
                             MapType& outputMap)
  {
    // Check compliance.
    Size size(costmap2d.getSizeInCellsX(), costmap2d.getSizeInCellsY());
    if ((outputMap.getSize() != size).any()) {
      ROS_ERROR("Costmap2D and output map have different sizes!");
      return false;
    }
    if (!outputMap.getStartIndex().isZero()) {
      ROS_ERROR("CostmapConverter::fromCostmap2D() does not support non-zero start indices!");
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

  /**
   * @brief Load the costmap2d into the specified layer.
   *
   * @param costmap2d : from this costmap type object
   * @param layer : name of the layer to insert
   * @param outputMap : to this map type object
   *
   * @warning this does not lock the costmap2d object, you should take care to do so from outside this function.
   **/
  bool addLayerFromCostmap2D(costmap_2d::Costmap2DROS& costmap2d,
                             const std::string& layer,
                             MapType& outputMap)
  {
    return addLayerFromCostmap2D(*(costmap2d.getCostmap()), layer, outputMap);
  }

  /**
   * @brief Initialise the output map at the robot pose of the underlying Costmap2DROS object.
   *
   * @param costmap2d : the underlying Costmap2DROS object
   * @param geometry: size of the subwindow to snapshot around the robot pose
   * @param outputMap : initialise the output map with the required parameters
   *
   * This needs some beyond just fixing the centre of the output map to the robot pose
   * itself. To avoid introducing error, you want to shift the centre so the
   * underlying costmap2d cells align with the newly created map object. This does
   * that here.
   *
   * @warning this does not lock the costmap2d object, you should take care to do so from
   * outside this function in scope with any addLayerFromCostmap2DAtRobotPose calls you wish to make.
   **/
  bool initializeFromCostmap2DAtRobotPose(costmap_2d::Costmap2DROS& costmap2d,
                                          const Length& geometry,
                                          MapType& outputMap)
  {
    /****************************************
    ** Properties
    ****************************************/
    const double resolution = costmap2d.getCostmap()->getResolution();

    /****************************************
    ** Get the Robot Pose Transform
    ****************************************/
    tf::Stamped<tf::Pose> tf_pose;
    if(!costmap2d.getRobotPose(tf_pose))
    {
      std::ostringstream error_message;
      error_message << "could not get robot pose, is it actually published?";
      ROS_ERROR_STREAM("CostmapConverter::" << error_message.str());
      return false;
      // throw std::runtime_error(error_message.str());
    }

    /****************************************
    ** Where is the New Costmap Origin?
    ****************************************/
    Position robot_position(tf_pose.getOrigin().x() , tf_pose.getOrigin().y());
    Position ros_map_origin(costmap2d.getCostmap()->getOriginX(), costmap2d.getCostmap()->getOriginY());
    Position new_cost_map_origin;

    // Note:
    //   You cannot directly use the robot pose as the new 'costmap centre'
    //   since the underlying grid is not necessarily exactly aligned with
    //   that (two cases to consider, rolling window and globally fixed).
    //
    // Relevant diagrams:
    //  - https://github.com/ethz-asl/grid_map

    // float versions of the cell co-ordinates, use static_cast<int> to get the indices
    Position robot_cell_position = (robot_position - ros_map_origin)/resolution;

    // if there is an odd number of cells
    //   centre of the new grid map in the centre of the current cell
    // if there is an even number of cells
    //   centre of the new grid map at the closest vertex between cells
    // of the current cell
    int number_of_cells_x = geometry.x()/resolution;
    int number_of_cells_y = geometry.y()/resolution;
    if ( number_of_cells_x % 2 ) { // odd
      new_cost_map_origin(0) = std::floor(robot_cell_position.x())*resolution + resolution/2.0 + ros_map_origin.x();
    } else {
      new_cost_map_origin(0) = std::round(robot_cell_position.x())*resolution + ros_map_origin.x();
    }
    if ( number_of_cells_y % 2 ) { // odd
      new_cost_map_origin(1) = std::floor(robot_cell_position.y())*resolution + resolution/2.0 + ros_map_origin.y();
    } else {
      new_cost_map_origin(1) = std::round(robot_cell_position.y())*resolution + ros_map_origin.y();
    }

    /****************************************
    ** Asserts
    ****************************************/
    // TODO check the robot pose is in the window
    // TODO check the geometry fits within the costmap2d window

    /****************************************
    ** Initialise the Output Map
    ****************************************/
    outputMap.setFrameId(costmap2d.getGlobalFrameID());
    outputMap.setTimestamp(ros::Time::now().toNSec());
    outputMap.setGeometry(geometry, resolution, new_cost_map_origin);
    return true;
  }

  /**
   * @brief Fill a layer in the output map with data from the subwindow centred at the robot pose
   *
   * @param costmap2d : the underlying Costmap2DROS object
   * @param layer : layer name to add
   * @param outputMap : the initialised and filled in map output map
   *
   * @warning this does not lock the costmap2d object, you should take care to do so from
   * outside this function in scope with any initializeFromCostmap2DAtRobotPose calls
   * you wish to make.
   **/
  bool addLayerFromCostmap2DAtRobotPose(costmap_2d::Costmap2DROS& costmap2d,
                                        const std::string& layer,
                                        MapType& outputMap)
  {
    /****************************************
    ** Asserts
    ****************************************/
    if ( outputMap.getResolution() != costmap2d.getCostmap()->getResolution()) {
      std::ostringstream error_message;
      error_message << "Costmap2D and output map have different resolutions!";
      ROS_ERROR_STREAM("CostmapConverter::" << error_message.str());
      return false;
    }
    // 1) would be nice to check the output map centre has been initialised where it should be
    //      i.e. the robot pose didn't move since or the initializeFrom wasn't called
    //    but this would mean listening to tf's again and anyway, it gets shifted to make sure
    //    the costmaps align, so these wouldn't be exactly the same anyway
    // 2) check the geometry fits inside the costmap2d subwindow is done below

    /****************************************
    ** Properties
    ****************************************/
    const double resolution = costmap2d.getCostmap()->getResolution();
    const Length geometry = outputMap.getLength();
    const Position position = outputMap.getPosition();

    /****************************************
    ** Copy Data
    ****************************************/
    bool is_valid_window = false;
    costmap_2d::Costmap2D costmap_subwindow;
    // TODO
    is_valid_window = costmap_subwindow.copyCostmapWindow(
                            *(costmap2d.getCostmap()),
                            position.x() - geometry.x() / 2.0, // subwindow_bottom_left_x
                            position.y() - geometry.y() / 2.0, // subwindow_bottom_left_y
                            geometry.x(),
                            geometry.y());
    if ( !is_valid_window ) {
      // handle differently - e.g. copy the internal part only and lethal elsewhere, but other parts would have to handle being outside too
      std::ostringstream error_message;
      error_message << "subwindow landed outside the costmap, aborting";
      ROS_ERROR_STREAM("CostmapConverter::" << error_message.str());
      return false;
      throw std::out_of_range(error_message.str());
    }
    addLayerFromCostmap2D(costmap_subwindow, layer, outputMap);
    return true;
  }

private:
  template <typename TranslationTable>
  void initializeCostTranslationTable(const TranslationTable& translation_table) {
    translation_table.apply(costTranslationTable_);
  }

  std::vector<typename MapType::DataType> costTranslationTable_;
};
/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace grid_map

#endif /* grid_map_ros_COSTMAP_2D_CONVERTER_HPP_ */
