/*
 * OctomapToGridmapDemo.cpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#include "grid_map_demos/OctomapToGridmapDemo.hpp"

#include <grid_map_octomap/GridMapOctomapConverter.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace grid_map_demos
{

OctomapToGridmapDemo::OctomapToGridmapDemo()
: Node("octomap_to_gridmap_demo"),
  map_(grid_map::GridMap({"elevation"}))
{
  readParameters();
  client_ = this->create_client<GetOctomapSrv>(octomapServiceTopic_);
  map_.setBasicLayers({"elevation"});
  gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(
      1).transient_local());
  octomapPublisher_ = this->create_publisher<OctomapMessage>(
    "octomap", rclcpp::QoS(
      1).transient_local());
}

OctomapToGridmapDemo::~OctomapToGridmapDemo()
{
}

bool OctomapToGridmapDemo::readParameters()
{
  this->declare_parameter("octomap_service_topic", std::string("/octomap_binary"));
  this->declare_parameter("min_x", NAN);
  this->declare_parameter("max_x", NAN);
  this->declare_parameter("min_y", NAN);
  this->declare_parameter("max_y", NAN);
  this->declare_parameter("min_z", NAN);
  this->declare_parameter("max_z", NAN);

  this->get_parameter("octomap_service_topic", octomapServiceTopic_);
  this->get_parameter("min_x", minX_);
  this->get_parameter("max_x", maxX_);
  this->get_parameter("min_y", minY_);
  this->get_parameter("max_y", maxY_);
  this->get_parameter("min_z", minZ_);
  this->get_parameter("max_z", maxZ_);
  return true;
}

void OctomapToGridmapDemo::convertAndPublishMap()
{
  rclcpp::Clock clock;

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "waiting for service to appear...");
  }

  auto request = std::make_shared<GetOctomapSrv::Request>();
  auto result_future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service: " << octomapServiceTopic_);
    return;
  }
  auto response = result_future.get();

  // creating octree
  octomap::OcTree * octomap = nullptr;
  octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(response->map);
  if (tree) {
    octomap = dynamic_cast<octomap::OcTree *>(tree);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call convert Octomap.");
    return;
  }

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;
  octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));
  if (!std::isnan(minX_)) {
    min_bound(0) = minX_;
  }
  if (!std::isnan(maxX_)) {
    max_bound(0) = maxX_;
  }
  if (!std::isnan(minY_)) {
    min_bound(1) = minY_;
  }
  if (!std::isnan(maxY_)) {
    max_bound(1) = maxY_;
  }
  if (!std::isnan(minZ_)) {
    min_bound(2) = minZ_;
  }
  if (!std::isnan(maxZ_)) {
    max_bound(2) = maxZ_;
  }
  bool res = grid_map::GridMapOctomapConverter::fromOctomap(
    *octomap, "elevation", map_, &min_bound,
    &max_bound);
  if (!res) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call convert Octomap.");
    return;
  }
  map_.setFrameId(response->map.header.frame_id);

  // Publish as grid map.
  auto gridMapMessage = grid_map::GridMapRosConverter::toMessage(map_);
  gridMapPublisher_->publish(std::move(gridMapMessage));

  // Also publish as an octomap msg for visualization
  OctomapMessage octomapMessage;
  octomap_msgs::fullMapToMsg(*octomap, octomapMessage);
  octomapMessage.header.frame_id = map_.getFrameId();

  std::unique_ptr<OctomapMessage> octomapMessagePtr(new
    OctomapMessage(octomapMessage));
  octomapPublisher_->publish(std::move(octomapMessagePtr));
}

}  // namespace grid_map_demos
