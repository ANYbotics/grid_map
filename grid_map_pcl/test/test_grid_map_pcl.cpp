/*
 * test_grid_map_pcl.cpp
 *
 *  Created on: Nov 4, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

// gtest
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

int argc;
char ** argv;

// Run all the tests that were declared with TEST()
int main(int _argc, char ** _argv)
{
  argc = _argc;
  argv = _argv;
  testing::InitGoogleTest(&_argc, _argv);
  srand(static_cast<int>(time(0)));
  return RUN_ALL_TESTS();
}
