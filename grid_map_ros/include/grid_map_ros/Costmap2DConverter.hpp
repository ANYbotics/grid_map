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
   * Initiliases the cost translation table with the default policy for the template
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

  void initializeFromCostmap2d(costmap_2d::Costmap2DROS& costmap2d, MapType& outputMap)
  {
    initializeFromCostmap2d(*(costmap2d.getCostmap()), outputMap);
    outputMap.setFrameId(costmap2d.getGlobalFrameID());
  }

  void initializeFromCostmap2d(const costmap_2d::Costmap2D& costmap2d, MapType& outputMap)
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
  bool addLayerFromCostmap2d(const costmap_2d::Costmap2D& costmap2d,
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
      ROS_ERROR("CostmapConverter::fromCostmap2d() does not support non-zero start indices!");
      return false;
    }
    // Copy data.
    // Reverse iteration is required because of different conventions
    // between Costmap2d and grid map.
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
   * @param outputMap : to this costmap type object
   * 
   * @warning this does not lock the costmap2d object, you should take care to do so from outside this function.
   **/
  bool addLayerFromCostmap2d(costmap_2d::Costmap2DROS& costmap2d,
                             const std::string& layer,
                             MapType& outputMap)
  {
    return addLayerFromCostmap2d(*(costmap2d.getCostmap()), layer, outputMap);
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
