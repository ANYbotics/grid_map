/*
 * GridMapVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "grid_map_visualization/GridMapVisualization.hpp"

namespace grid_map_visualization
{

GridMapVisualization::GridMapVisualization(const std::string & parameterName)
: visualizationsParameter_(parameterName),
  isSubscribed_(false)
{
  node_ptr = std::make_shared<rclcpp::Node>("grid_map_visualization");
  factory_ = std::make_shared<VisualizationFactory>(node_ptr);

  RCLCPP_INFO(node_ptr->get_logger(), "Grid map visualization node_ptr started.");
  readParameters();

  activityCheckTimer_ = node_ptr->create_wall_timer(
    std::chrono::duration<double>(1.0 / activityCheckRate_),
    std::bind(&GridMapVisualization::updateSubscriptionCallback, this));
  initialize();
}

GridMapVisualization::~GridMapVisualization()
{
}

bool GridMapVisualization::readParameters()
{
  node_ptr->declare_parameter("grid_map_topic", std::string("/grid_map"));
  node_ptr->declare_parameter("activity_check_rate", 2.0);
  node_ptr->declare_parameter(visualizationsParameter_, std::vector<std::string>());

  node_ptr->get_parameter("grid_map_topic", mapTopic_);
  node_ptr->get_parameter("activity_check_rate", activityCheckRate_);

  assert(activityCheckRate_);

  // Configure the visualizations from a configuration stored on the parameter server.
  std::vector<std::string> config;
  if (!node_ptr->get_parameter(visualizationsParameter_, config)) {
    RCLCPP_WARN(
      node_ptr->get_logger(),
      "Could not load the visualizations configuration from parameter %s,are you sure it"
      "was pushed to the parameter server? Assuming that you meant to leave it empty.",
      visualizationsParameter_.c_str());
    return false;
  }

  std::unordered_set<std::string> config_check;

  // Iterate over all visualizations (may be just one),
  for (auto name : config) {
    std::string type;

    // Check for name collisions within the list itself.
    if (config_check.find(name) == config_check.end()) {
      config_check.insert(name);
    } else {
      RCLCPP_ERROR(
        node_ptr->get_logger(),
        "%s: A visualization with the name '%s' already exists.",
        visualizationsParameter_.c_str(), name.c_str());
      return false;
    }

    node_ptr->declare_parameter(name + ".type");
    try {
      if (!node_ptr->get_parameter(name + ".type", type)) {
        RCLCPP_ERROR(
          node_ptr->get_logger(),
          "%s: Could not add a visualization because no type was given",
          name.c_str());
        return false;
      }
    } catch (const rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(
        node_ptr->get_logger(),
        "Could not add %s visualization, because the %s.type parameter is not a string.",
        name.c_str(), name.c_str());
      return false;
    }

    // Make sure the visualization has a valid type.
    if (!factory_->isValidType(type)) {
      RCLCPP_ERROR(
        node_ptr->get_logger(),
        "Could not add %s visualization, no visualization of type '%s' found.",
        name.c_str(), type.c_str());
      return false;
    }

    auto visualization = factory_->getInstance(type, name);
    visualization->readParameters();
    visualizations_.push_back(visualization);
    RCLCPP_INFO(
      node_ptr->get_logger(), "%s: Configured visualization of type '%s' with name '%s'.",
      visualizationsParameter_.c_str(), type.c_str(), name.c_str());
  }
  return true;
}

bool GridMapVisualization::initialize()
{
  for (auto & visualization : visualizations_) {
    visualization->initialize();
  }

  updateSubscriptionCallback();
  RCLCPP_INFO(node_ptr->get_logger(), "Grid map visualization initialized.");
  return true;
}

void GridMapVisualization::updateSubscriptionCallback()
{
  bool isActive = false;

  for (auto & visualization : visualizations_) {
    if (visualization->isActive()) {
      isActive = true;
      break;
    }
  }

  if (!isSubscribed_ && isActive) {
    mapSubscriber_ = node_ptr->create_subscription<grid_map_msgs::msg::GridMap>(
      mapTopic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&GridMapVisualization::callback, this, std::placeholders::_1));

    isSubscribed_ = true;
    RCLCPP_DEBUG(node_ptr->get_logger(), "Subscribed to grid map at '%s'.", mapTopic_.c_str());
  }
  if (isSubscribed_ && !isActive) {
    mapSubscriber_.reset();
    isSubscribed_ = false;
    RCLCPP_DEBUG(node_ptr->get_logger(), "Cancelled subscription to grid map.");
  }
}

void GridMapVisualization::callback(const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  RCLCPP_DEBUG(
    node_ptr->get_logger(),
    "Grid map visualization received a map (timestamp %f) for visualization.",
    rclcpp::Time(message->header.stamp).seconds());
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*message, map);

  for (auto & visualization : visualizations_) {
    visualization->visualize(map);
  }
}

}  // namespace grid_map_visualization
