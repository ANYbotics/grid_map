/*
 * MapRegionVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__MAPREGIONVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__MAPREGIONVISUALIZATION_HPP_

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <string>

namespace grid_map_visualization
{

/*!
 * Visualization of the region of the grid map as border line.
 */
class MapRegionVisualization : public VisualizationBase
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param name the name of the visualization.
   */
  MapRegionVisualization(ros::NodeHandle & nodeHandle, const std::string & name);

  /*!
   * Destructor.
   */
  virtual ~MapRegionVisualization();

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
  //! Marker to be published.
  visualization_msgs::Marker marker_;

  //! Number of vertices of the map region visualization.
  const unsigned int nVertices_;

  //! Color of the map region visualization.
  std_msgs::ColorRGBA color_;

  //! Line width of the map region marker [m].
  double lineWidth_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__MAPREGIONVISUALIZATION_HPP_
