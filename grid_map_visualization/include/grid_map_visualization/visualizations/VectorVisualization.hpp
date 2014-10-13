/*
 * VectorVisualization.hpp
 *
 *  Created on: Sep 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <grid_map/GridMap.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

// STD
#include <vector>

namespace grid_map_visualization {

class VectorVisualization
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  VectorVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~VectorVisualization();

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

  //! ROS publisher of the marker for the vectors.
  ros::Publisher markerPublisher_;

  //! Marker to be published.
  visualization_msgs::Marker marker_;

  //! Types that are transformed to vectors.
  std::vector<std::string> types_;

  //! Type that is the position of the vectors.
  std::string positionType_;

  //! Scaling of the vectors.
  double scale_;

  //! Width of the line markers [m].
  double lineWidth_;

  //! Color of the vectors.
  std_msgs::ColorRGBA color_;
};

} /* namespace */
