/*
 * tutorial_demo_node.cpp
 *
 *  Created on: Mar 17, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include <grid_map/grid_map.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

using namespace std;
using namespace grid_map;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_tutorial_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  GridMap map({"original", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).", map.getLength().x(),
           map.getLength().y(), map.getSize()(0), map.getSize()(1));

  ros::Rate rate(30.0);
  while (nh.ok()) {

    // Add height and surface normal (iterating through grid map).
    for (grid_map::GridMapIterator it(map); !it.isPassedEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      double time = ros::Time::now().toSec();
      map.at("original", *it) = -0.04 + 0.2 * sin(3.0 * time + 5.0 * position.y()) * position.x();
      Eigen::Vector3d normal(-0.2 * sin(3.0 * time + 5.0 * position.y()), -position.x() * cos(3.0 * time + 5.0 * position.y()), 1.0);
      normal.normalize();
      map.at("normal_x", *it) = normal.x();
      map.at("normal_y", *it) = normal.y();
      map.at("normal_z", *it) = normal.z();
    }

    // Add noise and outliers (using Eigen operators).
    map.add("std", Matrix::Random(map.getSize()(0), map.getSize()(1)) * 0.015);
    map.add("raw", map.get("original") + map.get("std"));

    for (unsigned int i = 0; i < 500; ++i) {
      Position randomPosition = Position::Random();
      if (map.isInside(randomPosition))
        map.atPosition("raw", randomPosition) = std::numeric_limits<float>::infinity();
    }

    for (grid_map::LineIterator iterator(map, Index(5, 6), Index(30, 30));
              !iterator.isPassedEnd(); ++iterator) {
      map.at("raw", *iterator) = 0.02;
    }

    // Filter values for submap (iterators).
    map.add("filtered", map.get("raw"));
    Position topLeftCorner(1.0, 0.4);
    grid_map::limitPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
    Index startIndex;
    map.getIndex(topLeftCorner, startIndex);
    ROS_INFO_ONCE("Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
             topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

    Size size = (Length(1.2, 0.8) / map.getResolution()).cast<int>();
    grid_map::SubmapIterator it(map, startIndex, size);
    for (; !it.isPassedEnd(); ++it) {
      Position currentPosition;
      map.getPosition(*it, currentPosition);
      double radius = 0.1;
      double mean = 0.0;
      double sumOfWeights = 0.0;

      // Compute weighted mean.
      for (grid_map::CircleIterator circleIt(map, currentPosition, radius);
          !circleIt.isPassedEnd(); ++circleIt) {
        if (!map.isValid(*circleIt, "raw")) continue;
        Position currentPositionInCircle;
        map.getPosition(*circleIt, currentPositionInCircle);

        // Computed weighted mean based on Euclidian distance.
        double distance = (currentPosition - currentPositionInCircle).norm();
        double weight = pow(radius - distance, 2);
        mean += weight * map.at("raw", *circleIt);
        sumOfWeights += weight;
      }

      map.at("filtered", *it) = mean / sumOfWeights;
    }

    // Show absolute difference.
    Matrix error = (map.get("filtered") - map.get("original")).cwiseAbs();
    map.add("error", error);

    // Publish grid map.
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    rate.sleep();
  }

  return 0;
}
