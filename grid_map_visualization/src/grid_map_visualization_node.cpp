/*
 * grid_map_visualization_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_visualization/GridMapVisualization.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<grid_map_visualization::GridMapVisualization>(
    "grid_map_visualizations");
  rclcpp::spin(node->nodePtr);
  rclcpp::shutdown();
  return 0;
}
