/*
 * simple_demo_node.cpp
 *
 *  Created on: Mar 17, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include <grid_map/grid_map.hpp>
#include <grid_map_msgs/GridMap.h>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

using namespace std;
using namespace grid_map;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nodeHandle("~");
  ros::Publisher publisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  vector<string> layerNames;
  layerNames.push_back("original");
  layerNames.push_back("std");
  GridMap map(layerNames);
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).", map.getLength().x(),
           map.getLength().y(), map.getSize()(0), map.getSize()(1));

  ros::Rate rate(30.0);
  while (nodeHandle.ok()) {

    // Add height (iterating through grid map).
    for (grid_map_core::GridMapIterator iterator(map); !iterator.isPassedEnd(); ++iterator) {
      Position position;
      map.getPosition(*iterator, position);
      double time = ros::Time::now().toSec();
      map.at("original", *iterator) = -0.04 + 0.2 * sin(3.0 * time + 5.0 * position.y()) * position.x();
    }

    // Add noise and outliers (using Eigen operators).
    map.get("std").setRandom();
    map.get("std") *= 0.015;
    map.add("raw", map.get("original") + map.get("std"));

    for (unsigned int i = 0; i < 500; ++i) {
      Position randomPosition = Position::Random();
      if (map.isInside(randomPosition))
        map.atPosition("raw", randomPosition) = std::numeric_limits<float>::infinity();
    }


    for (grid_map_core::LineIterator iterator(map, Index(5, 6), Index(30, 30));
              !iterator.isPassedEnd(); ++iterator) {
      map.at("raw", *iterator) = 0.02;
    }

    // Filter values for submap (iterators).
    map.add("filtered", map.get("raw"));
    Position topLeftCorner(1.0, 0.4);
    grid_map_core::limitPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
    Index startIndex;
    map.getIndex(topLeftCorner, startIndex);
    ROS_INFO("Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
             topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

    Size size = (Length(1.2, 0.8) / map.getResolution()).cast<int>();
    grid_map_core::SubmapIterator submapIterator(map, startIndex, size);
    for (; !submapIterator.isPassedEnd(); ++submapIterator) {
      Position currentPosition;
      map.getPosition(*submapIterator, currentPosition);
      double radius = 0.1;
      double mean = 0.0;
      double sumOfWeights = 0.0;

      // Compute weighted mean.
      for (grid_map_core::CircleIterator circleIterator(map, currentPosition, radius);
          !circleIterator.isPassedEnd(); ++circleIterator) {
        if (!map.isValid(*circleIterator, "raw")) continue;
        Position currentPositionInCircle;
        map.getPosition(*circleIterator, currentPositionInCircle);

        // Computed weighted mean based on Euclidian distance.
        double distance = (currentPosition - currentPositionInCircle).norm();
        double weight = pow(radius - distance, 2);
        mean += weight * map.at("raw", *circleIterator);
        sumOfWeights += weight;
      }

      map.at("filtered", *submapIterator) = mean / sumOfWeights;
    }

    // Show absolute difference.
    Matrix error = (map.get("filtered") - map.get("original")).cwiseAbs();
    map.add("error", error);

    // Publish grid map.
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    map.toMessage(message);
    publisher.publish(message);
    ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    rate.sleep();
  }

  return 0;
}
