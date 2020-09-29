/*
 * PointCloudOccupancyGrid.hpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__OCCUPANCYGRIDVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__OCCUPANCYGRIDVISUALIZATION_HPP_

#include <grid_map_core/GridMap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization
{

class OccupancyGridVisualization : public VisualizationBase
{
public:
  /*!
   * Constructor.
   * @param name the name of the visualization.
   */
  explicit OccupancyGridVisualization(const std::string & name, rclcpp::Node::SharedPtr nodePtr);

  /*!
   * Destructor.
   */
  virtual ~OccupancyGridVisualization();

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

  //! Minimum and maximum value of the grid map data
  // (used to normalize the cell data in [min, max]).
  float dataMin_, dataMax_;

  //! ROS publisher.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__OCCUPANCYGRIDVISUALIZATION_HPP_
