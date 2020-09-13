/*
 * octomap_to_gridmap_demo_node.cpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "grid_map_demos/OctomapToGridmapDemo.hpp"

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);

  auto octomapToGridmapDemo = std::make_shared<grid_map_demos::OctomapToGridmapDemo>();

  auto sleep_duration = rclcpp::Duration::from_seconds(2.0);
  rclcpp::sleep_for(std::chrono::nanoseconds(sleep_duration.nanoseconds()));

  rclcpp::Rate r(0.1);  // 1 hz
  while (rclcpp::ok()) {
    octomapToGridmapDemo->convertAndPublishMap();
    rclcpp::spin_some(octomapToGridmapDemo);
    r.sleep();
  }
  return 0;
}
