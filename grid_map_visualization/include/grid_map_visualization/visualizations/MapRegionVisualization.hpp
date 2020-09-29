/*
 * MapRegionVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__MAPREGIONVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__MAPREGIONVISUALIZATION_HPP_

#include <grid_map_core/GridMap.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

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
   * @param name the name of the visualization.
   */
  explicit MapRegionVisualization(const std::string & name, rclcpp::Node::SharedPtr nodePtr);

  /*!
   * Destructor.
   */
  virtual ~MapRegionVisualization();

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
  //! Marker to be published.
  visualization_msgs::msg::Marker marker_;

  //! Number of vertices of the map region visualization.
  const unsigned int nVertices_;

  //! Color of the map region visualization.
  std_msgs::msg::ColorRGBA color_;

  //! Line width of the map region marker [m].
  double lineWidth_;

  //! ROS publisher.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__MAPREGIONVISUALIZATION_HPP_
