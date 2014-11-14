/*
 * GridMapExample.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_example/GridMapExample.hpp"
#include <grid_map_msg/GridMap.h>
#include <grid_map_lib/iterators/GridMapIterator.hpp>
#include <grid_map_lib/iterators/SubmapIterator.hpp>
#include <grid_map_lib/iterators/CircleIterator.hpp>
#include <grid_map/Polygon.hpp>
#include <grid_map_lib/iterators/PolygonIterator.hpp>
#include <grid_map_lib/iterators/LineIterator.hpp>

// ROS
#include <geometry_msgs/PolygonStamped.h>

// Eigen
#include <Eigen/Core>

using namespace std;
using namespace ros;

namespace grid_map_example {

GridMapExample::GridMapExample(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(vector<string>({"type"}))
{
  ROS_INFO("Grid map example node started.");
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msg::GridMap>("grid_map", 1, true);
  polygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);

  // Setting up map.
  map_.setGeometry(Eigen::Array2d(1.0, 1.0), 0.05, Eigen::Vector2d(0.0, 0.0));
  map_.setFrameId("map");

  publish();
  ros::Duration duration(2.0);
  duration.sleep();

  demoGridMapIterator();
  demoSubmapIterator();
  demoCircleIterator();
  demoLineIterator();
  demoPolygonIterator();
}

GridMapExample::~GridMapExample() {}

void GridMapExample::demoGridMapIterator()
{
  ROS_INFO("Running grid map iterator demo.");
  map_.clear();
  publish();

  for (grid_map_lib::GridMapIterator iterator(map_); !iterator.isPassedEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.01);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void GridMapExample::demoSubmapIterator()
{
  ROS_INFO("Running submap iterator demo.");
  map_.clear();
  publish();

  Eigen::Array2i submapStartIndex(3, 5);
  Eigen::Array2i submapBufferSize(12, 7);

  for (grid_map_lib::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
      !iterator.isPassedEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void GridMapExample::demoCircleIterator()
{
  ROS_INFO("Running circle iterator demo.");
  map_.clear();
  publish();

  Eigen::Vector2d center(0.0, -0.15);
  double radius = 0.4;

  for (grid_map_lib::CircleIterator iterator(map_, center, radius);
      !iterator.isPassedEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void GridMapExample::demoLineIterator()
{
  ROS_INFO("Running line iterator demo.");
  map_.clear();
  publish();

  Eigen::Array2i start(18, 2);
  Eigen::Array2i end(2, 13);

  for (grid_map_lib::LineIterator iterator(map_, start, end);
      !iterator.isPassedEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void GridMapExample::demoPolygonIterator()
{
  ROS_INFO("Running polygon iterator demo.");
  map_.clear();
  publish();

  grid_map::Polygon polygon;
  polygon.setFrameId(map_.getFrameId());
  polygon.addVertex(Eigen::Vector2d( 0.480,  0.000));
  polygon.addVertex(Eigen::Vector2d( 0.164,  0.155));
  polygon.addVertex(Eigen::Vector2d( 0.116,  0.500));
  polygon.addVertex(Eigen::Vector2d(-0.133,  0.250));
  polygon.addVertex(Eigen::Vector2d(-0.480,  0.399));
  polygon.addVertex(Eigen::Vector2d(-0.316,  0.000));
  polygon.addVertex(Eigen::Vector2d(-0.480, -0.399));
  polygon.addVertex(Eigen::Vector2d(-0.133, -0.250));
  polygon.addVertex(Eigen::Vector2d( 0.116, -0.500));
  polygon.addVertex(Eigen::Vector2d( 0.164, -0.155));
  polygon.addVertex(Eigen::Vector2d( 0.480,  0.000));

  geometry_msgs::PolygonStamped polygonMsg;
  polygon.toMessage(polygonMsg);
  polygonPublisher_.publish(polygonMsg);

  for (grid_map_lib::PolygonIterator iterator(map_, polygon);
      !iterator.isPassedEnd(); ++iterator) {
    map_.at("type", *iterator) = 1.0;
    publish();
    ros::Duration duration(0.02);
    duration.sleep();
  }

  ros::Duration duration(1.0);
  duration.sleep();
}

void GridMapExample::publish()
{
  map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_msg::GridMap message;
  map_.toMessage(message);
  gridMapPublisher_.publish(message);
  ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}

} /* namespace */
