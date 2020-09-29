/*
 * GridCellsVisualization.hpp
 *
 *  Created on: Mar 28, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__GRIDCELLSVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__GRIDCELLSVISUALIZATION_HPP_

#include <grid_map_core/GridMap.hpp>
#include <nav_msgs/msg/grid_cells.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <string>

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"


namespace grid_map_visualization
{

class GridCellsVisualization : public VisualizationBase
{
public:
  /*!
   * Constructor.
   * @param name the name of the visualization.
   */
  explicit GridCellsVisualization(const std::string & name, rclcpp::Node::SharedPtr nodePtr);

  /*!
   * Destructor.
   */
  virtual ~GridCellsVisualization();

  /*!
   * Read parameters from ROS.
   * @param config the parameters as XML.
   * @return true if successful.
   */
  bool readParameters() override;

  /*!
   * Initialization.
   */
  bool initialize() override;

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap & map) override;

private:
  //! Type that is transformed to the occupancy grid.
  std::string layer_;

  //! Values that are between lower and upper threshold are shown.
  float lowerThreshold_, upperThreshold_;

  //! ROS publisher.
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr publisher_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__GRIDCELLSVISUALIZATION_HPP_
