/*
 * interpolation_demo_node.cpp
 *
 *  Created on: Mar 16, 2020
 *      Author: jelavice
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "grid_map_demos/InterpolationDemo.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grid_map_demos::InterpolationDemo>());
  rclcpp::shutdown();
  return 0;
}
