/*
 * VisualizationBase.hpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__VISUALIZATIONBASE_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__VISUALIZATIONBASE_HPP_


#include <grid_map_core/GridMap.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>

namespace grid_map_visualization
{

class VisualizationBase : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   * @param name the name of the visualization.
   */
  explicit VisualizationBase(const std::string & name);

  /*!
   * Destructor.
   */
  virtual ~VisualizationBase();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  virtual bool readParameters() = 0;

  /*!
   * Initialization.
   */
  virtual bool initialize() = 0;

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  virtual bool visualize(const grid_map::GridMap & map) = 0;

  /*!
   * Checks if visualization is active (if somebody has actually subscribed).
   * @return true if active, false otherwise.
   */
  bool isActive() const;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__VISUALIZATIONBASE_HPP_
