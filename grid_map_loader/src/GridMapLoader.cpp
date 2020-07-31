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
#include <memory>
#include <string>

namespace grid_map_loader
{

GridMapLoader::GridMapLoader()
: Node("grid_map_loader")
{
  readParameters();

  rclcpp::QoS custom_qos(rclcpp::KeepLast(1));  // Buffer only last published message.
  custom_qos.transient_local();  // Persist messages for “late-joining” subscriptions.
  custom_qos.lifespan(duration_);  // Expiration duration of persisted messasges.

  publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(publishTopic_, custom_qos);

  if (!load()) {return;}
  publish();
}

GridMapLoader::~GridMapLoader()
{
}

bool GridMapLoader::readParameters()
{
  double durationInSec;
  this->declare_parameter("bag_topic", std::string("/grid_map"));
  this->declare_parameter("publish_topic", std::string("/grid_map"));
  this->declare_parameter("file_path", std::string());
  this->declare_parameter("duration", rclcpp::ParameterValue(5.0));

  this->get_parameter("bag_topic", bagTopic_);
  this->get_parameter("publish_topic", publishTopic_);
  this->get_parameter("file_path", filePath_);
  this->get_parameter("duration", durationInSec);

  duration_ = rclcpp::Duration::from_seconds(durationInSec);
  return true;
}

bool GridMapLoader::load()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Loading grid map from path " << filePath_ << ".");
  return grid_map::GridMapRosConverter::loadFromBag(filePath_, bagTopic_, map_);
}

void GridMapLoader::publish()
{
  grid_map_msgs::msg::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  publisher_->publish(std::make_unique<grid_map_msgs::msg::GridMap>(message));
  rclcpp::sleep_for(std::chrono::nanoseconds(duration_.nanoseconds()));
}
}  // namespace grid_map_loader
