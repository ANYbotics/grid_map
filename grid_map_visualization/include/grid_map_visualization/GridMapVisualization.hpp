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
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

namespace grid_map_visualization
{

/*!
 * Visualizes a grid map by publishing different topics that can be viewed in Rviz.
 * Implemented as a lifecycle node so the visualization pipeline can be activated
 * only when needed, saving CPU when no operator is monitoring.
 */
class GridMapVisualization : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /*!
   * Constructor.
   * @param parameterName The config parameter name.
   */
  explicit GridMapVisualization(const std::string & parameterName);

  /*!
   * Destructor.
   */
  virtual ~GridMapVisualization() = default;

  /*!
   * Reads parameters and sets up visualizations.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state);

  /*!
   * Creates the grid map subscription and starts processing.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state);

  /*!
   * Destroys the grid map subscription and stops processing.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);

  /*!
   * Cleans up visualizations and factory.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);

  /*!
   * Shuts down the node.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

  /*!
   * Callback function for the grid map.
   * @param message the grid map message to be visualized.
   */
  void callback(const grid_map_msgs::msg::GridMap::SharedPtr message);

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

  //! If the grid map subscriber uses Transient Local durability.
  bool isGridMapSubLatched_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__GRIDMAPVISUALIZATION_HPP_
