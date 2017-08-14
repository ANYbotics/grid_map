/*
 * filters_demo_node.cpp
 *
 *  Created on: Aug 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
//#include <grid_map_filters/grid_map_ros.hpp>
#include <filters/filter_chain.h>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_filters_demo");
  ros::NodeHandle nodeHandle("~");
  ros::Publisher publisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Setup filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
  if (!filterChain.configure("grid_map_filters", nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    return false;
  }

  // Create input grid map.
  GridMap inputMap({"elevation"});
  inputMap.setFrameId("map");
  inputMap.setGeometry(Length(0.7, 0.7), 0.01, Position(0.0, 0.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    inputMap.getLength().x(), inputMap.getLength().y(),
    inputMap.getSize()(0), inputMap.getSize()(1),
    inputMap.getPosition().x(), inputMap.getPosition().y(), inputMap.getFrameId().c_str());
  inputMap["elevation"].setRandom();
  inputMap["elevation"] *= 0.1;

  while (nodeHandle.ok()) {

    // Publish original grid map.
    inputMap.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(inputMap, message);
    publisher.publish(message);
    ros::Duration(1.0).sleep();

    // Apply filter chain.
    GridMap outputMap;
    if (!filterChain.update(inputMap, outputMap)) {
      ROS_ERROR("Could not update the grid map filter chain!");
      break;
    }

    // Publish filtered output grid map.
    outputMap.setTimestamp(ros::Time::now().toNSec());
    GridMapRosConverter::toMessage(outputMap, message);
    publisher.publish(message);
    ros::Duration(1.0).sleep();

//    // Work with temporary inputMap in a loop.
//    GridMap tempMap(map);
//    Rate rate(10.0);
//    ros::Time startTime = ros::Time::now();
//    ros::Duration duration(0.0);
//
//    while (duration <= ros::Duration(10.0)) {
//      ros::Time time = ros::Time::now();
//      duration = time - startTime;
//
//      // Change position of the map with either the `move` or `setPosition` method.
//      const double t = duration.toSec();
//      Position newPosition = 0.03 * t * Position(cos(t), sin(t));
//
//      if (useMoveMethod) {
//        tempMap.move(newPosition);
//      } else {
//        tempMap.setPosition(newPosition);
//      }
//
//      // Publish grid map.
//      tempMap.setTimestamp(time.toNSec());
//      grid_map_msgs::GridMap message;
//      GridMapRosConverter::toMessage(tempMap, message);
//      publisher.publish(message);
//      ROS_DEBUG("Grid map (duration %f) published with new position [%f, %f].",
//                duration.toSec(), tempMap.getPosition().x(), tempMap.getPosition().y());
//      rate.sleep();
//    }
//
//    useMoveMethod = !useMoveMethod;
  }

  return 0;
}
