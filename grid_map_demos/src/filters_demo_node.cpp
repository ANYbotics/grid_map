/*
 * filters_demo_node.cpp
 *
 *  Created on: Aug 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_demos/FiltersDemo.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_filters_demo");
  ros::NodeHandle nodeHandle("~");
  bool success;
  grid_map_demos::FiltersDemo filtersDemo(nodeHandle, success);
  if (success) ros::spin();
  return 0;
}





///*
// * filters_demo_node.cpp
// *
// *  Created on: Aug 14, 2017
// *      Author: Peter Fankhauser
// *   Institute: ETH Zurich, ANYbotics
// */
//
//#include <filters/filter_chain.h>
//#include <grid_map_core/grid_map_core.hpp>
//#include <grid_map_ros/grid_map_ros.hpp>
//#include <grid_map_msgs/GridMap.h>
//#include <ros/ros.h>
//
//using namespace grid_map;
//
//class GridMapFilterChain
//
//
//(const grid_map_msgs::GridMapPtr& message)
//{
//  GridMap inputMap;
//  GridMapRosConverter::fromMessage(message, inputMap);
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
//}
//
//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "grid_map_filters_demo");
//  ros::NodeHandle nodeHandle("~");
//  ros::Subscriber subscriber = nodeHandle.subscribe("grid_map_in", 1000, chatterCallback);
//  ros::Publisher publisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map_out", 1, true);
//
//  // Setup filter chain.
//  filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
//  if (!filterChain.configure("grid_map_filters", nodeHandle)) {
//    ROS_ERROR("Could not configure the filter chain!");
//    return false;
//  }
//
//  // Create input grid map.
//  GridMap inputMap({"elevation"});
//  inputMap.setFrameId("map");
//  inputMap.setGeometry(Length(0.7, 0.7), 0.01, Position(0.0, 0.0));
//  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
//    inputMap.getLength().x(), inputMap.getLength().y(),
//    inputMap.getSize()(0), inputMap.getSize()(1),
//    inputMap.getPosition().x(), inputMap.getPosition().y(), inputMap.getFrameId().c_str());
//  inputMap["elevation"].setRandom();
//  inputMap["elevation"] *= 0.01;
//  for (grid_map::CircleIterator iterator(inputMap, Position(0.0, 0.0), 0.15); !iterator.isPastEnd(); ++iterator) {
//    inputMap.at("elevation", *iterator) = 0.1;
//  }
//
//  while (nodeHandle.ok()) {
//
//    // Publish original grid map.
//    inputMap.setTimestamp(ros::Time::now().toNSec());
//    grid_map_msgs::GridMap message;
//    GridMapRosConverter::toMessage(inputMap, message);
//    publisher.publish(message);
//    ros::Duration(1.0).sleep();
//
//    // Apply filter chain.
//    GridMap outputMap;
//    if (!filterChain.update(inputMap, outputMap)) {
//      ROS_ERROR("Could not update the grid map filter chain!");
//      break;
//    }
//
//    // Publish filtered output grid map.
//    outputMap.setTimestamp(ros::Time::now().toNSec());
//    GridMapRosConverter::toMessage(outputMap, message);
//    publisher.publish(message);
//    ros::Duration(1.0).sleep();
//  }
//
//  return 0;
//}
