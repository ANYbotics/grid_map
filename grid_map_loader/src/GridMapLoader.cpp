/*
 * GridMapLoader.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#include "grid_map_loader/GridMapLoader.hpp"
#include <grid_map_msgs/msg/grid_map.hpp>

#include <chrono>
#include <string>
#include <utility>

namespace grid_map_loader
{

GridMapLoader::GridMapLoader()
: Node("grid_map_loader")
{
  readParameters();

  rclcpp::QoS custom_qos(10);  // initialize to default

  if (qos_transient_local_) {
    custom_qos.transient_local();  // Persist messages for “late-joining” subscriptions.
  }

  publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(publishTopic_, custom_qos);

  if (!load()) {return;}
  publish();
}

GridMapLoader::~GridMapLoader()
{
}

bool GridMapLoader::readParameters()
{
  this->declare_parameter("bag_topic", std::string("/grid_map"));
  this->declare_parameter("publish_topic", std::string("/grid_map"));
  this->declare_parameter("file_path", std::string());
  this->declare_parameter("duration", rclcpp::ParameterValue(5.0));
  this->declare_parameter("qos_transient_local", rclcpp::ParameterValue(true));

  this->get_parameter("bag_topic", bagTopic_);
  this->get_parameter("publish_topic", publishTopic_);
  this->get_parameter("file_path", filePath_);
  this->get_parameter("duration", durationInSec);
  this->get_parameter("qos_transient_local", qos_transient_local_);

  return true;
}

bool GridMapLoader::load()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Loading grid map from path " << filePath_ << ".");
  return grid_map::GridMapRosConverter::loadFromBag(filePath_, bagTopic_, map_);
}

void GridMapLoader::publish()
{
  auto message = grid_map::GridMapRosConverter::toMessage(map_);
  publisher_->publish(std::move(message));

  if (durationInSec > 0) {
    auto sleep_duration = rclcpp::Duration::from_seconds(durationInSec);
    rclcpp::sleep_for(std::chrono::nanoseconds(sleep_duration.nanoseconds()));
  }
}
}  // namespace grid_map_loader
