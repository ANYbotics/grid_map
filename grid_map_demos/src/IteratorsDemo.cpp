/*
 * IteratorsDemo.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_demos/IteratorsDemo.hpp"

// ROS
#include <geometry_msgs/PolygonStamped.h>

using namespace std;
using namespace ros;
using namespace grid_map;

namespace grid_map_demos {

IteratorsDemo::IteratorsDemo(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(vector<string>({"type"}))
{
  ROS_INFO("Grid map iterators demo node started.");
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  polygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);

  // Setting up map.
  map_.setGeometry(Length(1.0, 1.0), 0.05, Position(0.0, 0.0));
  map_.setFrameId("map");

  publish();
  ros::Duration duration(2.0);
  duration.sleep();

  demoGridMapIterator();
  demoSubmapIterator();
  demoCircleIterator();
  demoEllipseIterator();
  demoSpiralIterator();
  demoLineIterator();
  demoPolygonIterator();
  demoSlidingWindowIterator();
}

IteratorsDemo::~IteratorsDemo() {}

void IteratorsDemo::demoGridMapIterator()
{
  ROS_INFO("Running grid map iterator demo.");
  map_.clearAll();
  publish();

  // Note: In this example we locally store a reference to the map data
  // for improved performance. See `iterator_benchmark.cpp` and the
  // README.md for a comparison and more information.
  grid_map::Matrix& data = map_["type"];
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    const int i = iterator.getLinearIndex();
    data(i) = 1.0;
    publish();
    ros::Duration duration(0.01);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoSubmapIterator()
{
  ROS_INFO("Running submap iterator demo.");
  map_.clearAll();
  publish();

  Index submapStartIndex(3, 5);
  Index submapBufferSize(12, 7);

  for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoCircleIterator()
{
  ROS_INFO("Running circle iterator demo.");
  map_.clearAll();
  publish();

  Position center(0.0, -0.15);
  double radius = 0.4;

  for (grid_map::CircleIterator iterator(map_, center, radius);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoEllipseIterator()
{
  ROS_INFO("Running ellipse iterator demo.");
  map_.clearAll();
  publish();

  Position center(0.0, -0.15);
  Length length(0.45, 0.9);

  for (grid_map::EllipseIterator iterator(map_, center, length, M_PI_4);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoSpiralIterator()
{
  ROS_INFO("Running spiral iterator demo.");
  map_.clearAll();
  publish();

  Position center(0.0, -0.15);
  double radius = 0.4;

  for (grid_map::SpiralIterator iterator(map_, center, radius);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoLineIterator()
{
  ROS_INFO("Running line iterator demo.");
  map_.clearAll();
  publish();

  Index start(18, 2);
  Index end(2, 13);

  for (grid_map::LineIterator iterator(map_, start, end);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void IteratorsDemo::demoPolygonIterator(const bool prepareForOtherDemos)
{
  ROS_INFO("Running polygon iterator demo.");
  map_.clearAll();
  if (prepareForOtherDemos) map_["type"].setZero();
  publish();

  grid_map::Polygon polygon;
  polygon.setFrameId(map_.getFrameId());
  polygon.addVertex(Position( 0.480,  0.000));
  polygon.addVertex(Position( 0.164,  0.155));
  polygon.addVertex(Position( 0.116,  0.500));
  polygon.addVertex(Position(-0.133,  0.250));
  polygon.addVertex(Position(-0.480,  0.399));
  polygon.addVertex(Position(-0.316,  0.000));
  polygon.addVertex(Position(-0.480, -0.399));
  polygon.addVertex(Position(-0.133, -0.250));
  polygon.addVertex(Position( 0.116, -0.500));
  polygon.addVertex(Position( 0.164, -0.155));
  polygon.addVertex(Position( 0.480,  0.000));

  geometry_msgs::PolygonStamped message;
  grid_map::PolygonRosConverter::toMessage(polygon, message);
  polygonPublisher_.publish(message);

  for (grid_map::PolygonIterator iterator(map_, polygon);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    if (!prepareForOtherDemos) {
      publish();
      ros::Duration duration(0.02);
      duration.sleep();
    }
  }

  if (!prepareForOtherDemos) {
    ros::Duration duration(1.0);
    duration.sleep();
  }
}

void IteratorsDemo::demoSlidingWindowIterator()
{
  ROS_INFO("Running sliding window iterator demo.");
  demoPolygonIterator(true);
  publish();
  const size_t windowSize = 3;
  const grid_map::SlidingWindowIterator::EdgeHandling edgeHandling = grid_map::SlidingWindowIterator::EdgeHandling::CROP;
  map_.add("copy", map_["type"]);

  for (grid_map::SlidingWindowIterator iterator(map_, "copy", edgeHandling, windowSize);
      !iterator.isPastEnd(); ++iterator) {
    map_.at("type", *iterator) = iterator.getData().meanOfFinites(); // Blurring.
    publish();

    // Visualize sliding window as polygon.
    grid_map::Polygon polygon;
    Position center;
    map_.getPosition(*iterator, center);
    const Length windowHalfLength(Length::Constant(0.5 * (double) windowSize * map_.getResolution()));
    polygon.addVertex(center + (Eigen::Array2d(-1.0,-1.0) * windowHalfLength).matrix());
    polygon.addVertex(center + (Eigen::Array2d(-1.0, 1.0) * windowHalfLength).matrix());
    polygon.addVertex(center + (Eigen::Array2d( 1.0, 1.0) * windowHalfLength).matrix());
    polygon.addVertex(center + (Eigen::Array2d( 1.0,-1.0) * windowHalfLength).matrix());
    polygon.setFrameId(map_.getFrameId());
    geometry_msgs::PolygonStamped message;
    grid_map::PolygonRosConverter::toMessage(polygon, message);
    polygonPublisher_.publish(message);

    ros::Duration duration(0.02);
    duration.sleep();
  }
}

void IteratorsDemo::publish()
{
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  gridMapPublisher_.publish(message);
  ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}

} /* namespace */
