/*
 * GridMapVisualization.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#ifndef GRID_MAP_VISUALIZATION__GRIDMAPVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__GRIDMAPVISUALIZATION_HPP_

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_visualization/visualizations/MapRegionVisualization.hpp>
#include <grid_map_visualization/visualizations/OccupancyGridVisualization.hpp>
#include <grid_map_visualization/visualizations/PointCloudVisualization.hpp>
#include <grid_map_visualization/visualizations/VectorVisualization.hpp>
#include <grid_map_visualization/visualizations/VisualizationFactory.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

namespace grid_map_visualization
{

/*!
 * Visualizes a grid map by publishing different topics that can be viewed in Rviz.
 */
class GridMapVisualization
{
public:
  /*!
   * Constructor.
   * @param parameterName The config parameter name.
   */
  explicit GridMapVisualization(const std::string & parameterName);

  /*!
   * Destructor.
   */
  virtual ~GridMapVisualization();

  /*!
   * Callback function for the grid map.
   * @param message the grid map message to be visualized.
   */
  void callback(const grid_map_msgs::msg::GridMap::SharedPtr message);

  //! ROS node shared pointer
  rclcpp::Node::SharedPtr nodePtr;

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

  /*!
   * Check if visualizations are active (subscribed to),
   * and accordingly cancels/activates the subscription to the
   * grid map to save bandwidth.
   * @param timerEvent the timer event.
   */
  void updateSubscriptionCallback();

  //! Parameter name of the visualizer configuration list.
  std::string visualizationsParameter_;

  //! ROS subscriber to the grid map.
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr mapSubscriber_;

  //! Topic name of the grid map to be visualized.
  std::string mapTopic_;

  //! List of visualizations.
  std::vector<std::shared_ptr<VisualizationBase>> visualizations_;

  //! Visualization factory.
  std::shared_ptr<VisualizationFactory> factory_;

  //! Timer to check the activity of the visualizations.
  rclcpp::TimerBase::SharedPtr activityCheckTimer_;

  //! Rate of checking the activity of the visualizations.
  double activityCheckRate_;

  //! If the grid map visualization is subscribed to the grid map.
  bool isSubscribed_;

  //! If the grid map subscriber is Transient local.
  bool isGridMapSubLatched_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__GRIDMAPVISUALIZATION_HPP_
