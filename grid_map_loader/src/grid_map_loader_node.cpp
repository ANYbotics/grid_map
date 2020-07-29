/*
 * grid_map_loader_node.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_loader/GridMapLoader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grid_map_loader::GridMapLoader>());
  rclcpp::shutdown();
  return 0;
}
