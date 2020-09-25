/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <memory>
#include <string>
#include <utility>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_pcl_loader_node");
  gm::setVerbosityLevelToDebugIfFlagSet(node);

  rclcpp::QoS custom_qos = rclcpp::QoS(1).transient_local();
  auto gridMapPub = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map_from_raw_pointcloud", custom_qos);

  grid_map::GridMapPclLoader gridMapPclLoader(node->get_logger());
  const std::string pathToCloud = gm::getPcdFilePath(node);
  gridMapPclLoader.loadParameters(gm::getParameterPath());
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  gm::processPointcloud(&gridMapPclLoader, node);

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(node));

  gm::saveGridMap(gridMap, node, gm::getMapRosbagTopic(node));

  // publish grid map
  auto msg = grid_map::GridMapRosConverter::toMessage(gridMap);
  gridMapPub->publish(std::move(msg));

  // run
  rclcpp::spin(node->get_node_base_interface());
  return EXIT_SUCCESS;
}
