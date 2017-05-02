/*
 * test_grid_map_octomap.cpp
 *
 *  Created on: May 1, 2017
 *      Author: Jeff Delmerico
 *	 Institute: University of Zürich, Robotics and Perception Group
 */

// gtest
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
