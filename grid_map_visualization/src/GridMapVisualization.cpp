/*
 * GridMapVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "grid_map_visualization/GridMapVisualization.hpp"

namespace grid_map_visualization
{

GridMapVisualization::GridMapVisualization(const std::string & parameterName)
: rclcpp_lifecycle::LifecycleNode("grid_map_visualization"),
  visualizationsParameter_(parameterName),
  isGridMapSubLatched_(false)
{
  declare_parameter("grid_map_topic", std::string("/grid_map"));
  declare_parameter(visualizationsParameter_, std::vector<std::string>());
  declare_parameter("transient_local", rclcpp::ParameterValue(false));
}

GridMapVisualization::CallbackReturn GridMapVisualization::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring grid map visualization.");

  if (!readParameters()) {
    return CallbackReturn::FAILURE;
  }
  if (!initialize()) {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Grid map visualization configured.");
  return CallbackReturn::SUCCESS;
}

GridMapVisualization::CallbackReturn GridMapVisualization::on_activate(
  const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  rclcpp::QoS qos_setting = rclcpp::SystemDefaultsQoS();
  if (isGridMapSubLatched_) {
    qos_setting = rclcpp::QoS(1).transient_local();
  }

  mapSubscriber_ = create_subscription<grid_map_msgs::msg::GridMap>(
    mapTopic_, qos_setting,
    std::bind(&GridMapVisualization::callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to grid map at '%s'.", mapTopic_.c_str());
  return CallbackReturn::SUCCESS;
}

GridMapVisualization::CallbackReturn GridMapVisualization::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  mapSubscriber_.reset();
  RCLCPP_INFO(get_logger(), "Cancelled subscription to grid map.");
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

GridMapVisualization::CallbackReturn GridMapVisualization::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  visualizations_.clear();
  factory_.reset();
  return CallbackReturn::SUCCESS;
}

GridMapVisualization::CallbackReturn GridMapVisualization::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  mapSubscriber_.reset();
  visualizations_.clear();
  factory_.reset();
  return CallbackReturn::SUCCESS;
}

bool GridMapVisualization::readParameters()
{
  get_parameter("grid_map_topic", mapTopic_);
  get_parameter("transient_local", isGridMapSubLatched_);

  auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
  factory_ = std::make_shared<VisualizationFactory>(node_ptr);

  // Configure the visualizations from a configuration stored on the parameter server.
  std::vector<std::string> config;
  if (!get_parameter(visualizationsParameter_, config)) {
    RCLCPP_WARN(
      get_logger(),
      "Could not load the visualizations configuration from parameter %s, are you sure it "
      "was pushed to the parameter server? Assuming that you meant to leave it empty.",
      visualizationsParameter_.c_str());
    return false;
  }

  std::unordered_set<std::string> config_check;

  // Iterate over all visualizations (may be just one).
  for (auto name : config) {
    std::string type;

    // Check for name collisions within the list itself.
    if (config_check.find(name) == config_check.end()) {
      config_check.insert(name);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "%s: A visualization with the name '%s' already exists.",
        visualizationsParameter_.c_str(), name.c_str());
      return false;
    }

    declare_parameter<std::string>(name + ".type");
    try {
      if (!get_parameter(name + ".type", type)) {
        RCLCPP_ERROR(
          get_logger(),
          "%s: Could not add a visualization because no type was given",
          name.c_str());
        return false;
      }
    } catch (const rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not add %s visualization, because the %s.type parameter is not a string.",
        name.c_str(), name.c_str());
      return false;
    }

    // Make sure the visualization has a valid type.
    if (!factory_->isValidType(type)) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not add %s visualization, no visualization of type '%s' found.",
        name.c_str(), type.c_str());
      return false;
    }

    auto visualization = factory_->getInstance(type, name);
    visualization->readParameters();
    visualizations_.push_back(visualization);
    RCLCPP_INFO(
      get_logger(), "%s: Configured visualization of type '%s' with name '%s'.",
      visualizationsParameter_.c_str(), type.c_str(), name.c_str());
  }
  return true;
}

bool GridMapVisualization::initialize()
{
  for (auto & visualization : visualizations_) {
    visualization->initialize();
  }
  RCLCPP_INFO(get_logger(), "Grid map visualization initialized.");
  return true;
}

void GridMapVisualization::callback(const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Grid map visualization received a map (timestamp %f) for visualization.",
    rclcpp::Time(message->header.stamp).seconds());
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*message, map);

  for (auto & visualization : visualizations_) {
    visualization->visualize(map);
  }
}

}  // namespace grid_map_visualization
