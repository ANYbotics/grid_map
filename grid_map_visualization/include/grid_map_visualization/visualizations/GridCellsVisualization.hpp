/*
 * GridCellsVisualization.hpp
 *
 *  Created on: Mar 28, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__GRIDCELLSVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__GRIDCELLSVISUALIZATION_HPP_

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include <ros/ros.h>

#include <string>

namespace grid_map_visualization
{

class GridCellsVisualization : public VisualizationBase
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param name the name of the visualization.
   */
  GridCellsVisualization(ros::NodeHandle & nodeHandle, const std::string & name);

  /*!
   * Destructor.
   */
  virtual ~GridCellsVisualization();

  /*!
   * Read parameters from ROS.
   * @param config the parameters as XML.
   * @return true if successful.
   */
  bool readParameters(XmlRpc::XmlRpcValue & config);

  /*!
   * Initialization.
   */
  bool initialize();

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap & map);

private:
  //! Type that is transformed to the occupancy grid.
  std::string layer_;

  //! Values that are between lower and upper threshold are shown.
  float lowerThreshold_, upperThreshold_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__GRIDCELLSVISUALIZATION_HPP_
