/*
 * grid_map_iterators_demo_node.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "grid_map_demos/IteratorsDemo.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grid_map_demos::IteratorsDemo>());
  rclcpp::shutdown();
  return 0;
}
