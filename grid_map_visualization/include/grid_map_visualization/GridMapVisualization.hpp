/*
 * GridMapVisualization.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

#include <grid_map_msg/GridMap.h>
#include <grid_map_visualization/visualizations/MapRegionVisualization.hpp>
#include <grid_map_visualization/visualizations/PointCloudVisualization.hpp>
#include <grid_map_visualization/visualizations/VectorVisualization.hpp>
#include <grid_map_visualization/visualizations/OccupancyGridVisualization.hpp>

// ROS
#include <ros/ros.h>

namespace grid_map_visualization {

/*!
 * Visualizes a grid map by publishing different topics that can be viewed in Rviz.
 */
class GridMapVisualization
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GridMapVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~GridMapVisualization();

  /*!
   * Callback function for the grid map.
   * @param message the grid map message to be visualized.
   */
  void callback(const grid_map_msg::GridMap& message);

 private:

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   * @return true if successful.
   */
  bool initialize();

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS subscriber to the grid map.
  ros::Subscriber mapSubscriber_;

  //! Topic name of the grid map to be visualized.
  std::string mapTopic_;

  //! Map region visualization.
  MapRegionVisualization mapRegionVisualization_;

  //! Visualizing map as point cloud.
  PointCloudVisualization pointCloudVisualization_;

  //! Visualizing data as vectors.
  VectorVisualization vectorVisualization_;

  //! Occupancy grid visualization.
  OccupancyGridVisualization occupancyGridVisualization_;
};

} /* namespace */
