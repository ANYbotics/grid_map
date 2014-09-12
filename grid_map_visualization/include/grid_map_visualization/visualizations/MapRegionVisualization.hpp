/*
 * MapRegionVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map/GridMap.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

namespace grid_map_visualization {

/*!
 * Visualization of the region of the grid map as border line.
 */
class MapRegionVisualization
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  MapRegionVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~MapRegionVisualization();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   */
  bool initialize();

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap& map);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS publisher of the marker array.
  ros::Publisher mapMarkerArrayPublisher_;

  //! Marker to be published.
  visualization_msgs::Marker marker_;

  //! Number of vertices of the map region visualization.
  const unsigned int nVertices_;

  //! Color of the map region visualization.
  std_msgs::ColorRGBA color_;

  //! Line width of the map region marker [m].
  double lineWidth_;

};

} /* namespace */
