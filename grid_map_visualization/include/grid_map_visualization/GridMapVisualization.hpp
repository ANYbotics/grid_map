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
#include <grid_map_visualization/visualizations/VisualizationFactory.hpp>
#include <grid_map_visualization/visualizations/MapRegionVisualization.hpp>
#include <grid_map_visualization/visualizations/PointCloudVisualization.hpp>
#include <grid_map_visualization/visualizations/VectorVisualization.hpp>
#include <grid_map_visualization/visualizations/OccupancyGridVisualization.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// STD
#include <string>
#include <vector>
#include <memory>

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

  /*!
   * Gets pointer to rclcpp::Node.
   * @return shared pointer to rclcpp::Node object.
   */
  rclcpp::Node::SharedPtr get_node_ptr();

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
  // void updateSubscriptionCallback(const ros::TimerEvent & timerEvent);

  //! ROS nodehandle.
  rclcpp::Node::SharedPtr nodeHandle_;

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
  // rclcpp::TimerBase activityCheckTimer_;

  //! Duration of checking the activity of the visualizations.
  // rclcpp::Duration activityCheckDuration_;

  //! If the grid map visualization is subscribed to the grid map.
  // bool isSubscribed_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__GRIDMAPVISUALIZATION_HPP_
