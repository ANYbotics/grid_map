/*
 * IteratorsDemo.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_demos/IteratorsDemo.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace grid_map_demos
{

IteratorsDemo::IteratorsDemo()
: Node("grid_map_iterators_demo"),
  map_(std::vector<std::string>({"type"}))
{
  RCLCPP_INFO(this->get_logger(), "Grid map iterators demo node started.");
  gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());
  polygonPublisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "polygon", rclcpp::QoS(1).transient_local());

  // Setting up map.
  map_.setGeometry(grid_map::Length(1.0, 1.0), 0.05, grid_map::Position(0.0, 0.0));
  map_.setFrameId("map");

  publish();
  rclcpp::sleep_for(std::chrono::seconds(2));

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
  RCLCPP_INFO(this->get_logger(), "Running grid map iterator demo.");
  map_.clearAll();
  publish();

  // Note: In this example we locally store a reference to the map data
  // for improved performance. See `iterator_benchmark.cpp` and the
  // README.md for a comparison and more information.
  grid_map::Matrix & data = map_["type"];
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    const int i = iterator.getLinearIndex();
    data(i) = 1.0;
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
}

void IteratorsDemo::demoSubmapIterator()
{
  RCLCPP_INFO(this->get_logger(), "Running submap iterator demo.");
  map_.clearAll();
  publish();

  grid_map::Index submapStartIndex(3, 5);
  grid_map::Index submapBufferSize(12, 7);

  for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
    !iterator.isPastEnd(); ++iterator)
  {
    map_.at("type", *iterator) = 1.0;
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
}

void IteratorsDemo::demoCircleIterator()
{
  RCLCPP_INFO(this->get_logger(), "Running circle iterator demo.");
  map_.clearAll();
  publish();

  grid_map::Position center(0.0, -0.15);
  double radius = 0.4;

  for (grid_map::CircleIterator iterator(map_, center, radius);
    !iterator.isPastEnd(); ++iterator)
  {
    map_.at("type", *iterator) = 1.0;
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
}

void IteratorsDemo::demoEllipseIterator()
{
  RCLCPP_INFO(this->get_logger(), "Running ellipse iterator demo.");
  map_.clearAll();
  publish();

  grid_map::Position center(0.0, -0.15);
  grid_map::Length length(0.45, 0.9);

  for (grid_map::EllipseIterator iterator(map_, center, length, M_PI_4);
    !iterator.isPastEnd(); ++iterator)
  {
    map_.at("type", *iterator) = 1.0;
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
}

void IteratorsDemo::demoSpiralIterator()
{
  RCLCPP_INFO(this->get_logger(), "Running spiral iterator demo.");
  map_.clearAll();
  publish();

  grid_map::Position center(0.0, -0.15);
  double radius = 0.4;

  for (grid_map::SpiralIterator iterator(map_, center, radius);
    !iterator.isPastEnd(); ++iterator)
  {
    map_.at("type", *iterator) = 1.0;
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
}

void IteratorsDemo::demoLineIterator()
{
  RCLCPP_INFO(this->get_logger(), "Running line iterator demo.");
  map_.clearAll();
  publish();

  grid_map::Index start(18, 2);
  grid_map::Index end(2, 13);

  for (grid_map::LineIterator iterator(map_, start, end);
    !iterator.isPastEnd(); ++iterator)
  {
    map_.at("type", *iterator) = 1.0;
    publish();
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
}

void IteratorsDemo::demoPolygonIterator(const bool prepareForOtherDemos)
{
  RCLCPP_INFO(this->get_logger(), "Running polygon iterator demo.");
  map_.clearAll();
  if (prepareForOtherDemos) {map_["type"].setZero();}
  publish();

  grid_map::Polygon polygon;
  polygon.setFrameId(map_.getFrameId());
  polygon.addVertex(grid_map::Position(0.480, 0.000));
  polygon.addVertex(grid_map::Position(0.164, 0.155));
  polygon.addVertex(grid_map::Position(0.116, 0.500));
  polygon.addVertex(grid_map::Position(-0.133, 0.250));
  polygon.addVertex(grid_map::Position(-0.480, 0.399));
  polygon.addVertex(grid_map::Position(-0.316, 0.000));
  polygon.addVertex(grid_map::Position(-0.480, -0.399));
  polygon.addVertex(grid_map::Position(-0.133, -0.250));
  polygon.addVertex(grid_map::Position(0.116, -0.500));
  polygon.addVertex(grid_map::Position(0.164, -0.155));
  polygon.addVertex(grid_map::Position(0.480, 0.000));

  geometry_msgs::msg::PolygonStamped message;
  grid_map::PolygonRosConverter::toMessage(polygon, message);
  polygonPublisher_->publish(message);

  for (grid_map::PolygonIterator iterator(map_, polygon);
    !iterator.isPastEnd(); ++iterator)
  {
    map_.at("type", *iterator) = 1.0;
    if (!prepareForOtherDemos) {
      publish();
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }

  if (!prepareForOtherDemos) {
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}

void IteratorsDemo::demoSlidingWindowIterator()
{
  RCLCPP_INFO(this->get_logger(), "Running sliding window iterator demo.");
  demoPolygonIterator(true);
  publish();
  const size_t windowSize = 3;
  const grid_map::SlidingWindowIterator::EdgeHandling edgeHandling =
    grid_map::SlidingWindowIterator::EdgeHandling::CROP;
  map_.add("copy", map_["type"]);

  for (grid_map::SlidingWindowIterator iterator(map_, "copy", edgeHandling, windowSize);
    !iterator.isPastEnd(); ++iterator)
  {
    map_.at("type", *iterator) = iterator.getData().meanOfFinites();  // Blurring.
    publish();

    // Visualize sliding window as polygon.
    grid_map::Polygon polygon;
    grid_map::Position center;
    map_.getPosition(*iterator, center);
    const grid_map::Length windowHalfLength(grid_map::Length::Constant(
        0.5 * static_cast<double>(windowSize) * map_.getResolution()));
    polygon.addVertex(center + (Eigen::Array2d(-1.0, -1.0) * windowHalfLength).matrix());
    polygon.addVertex(center + (Eigen::Array2d(-1.0, 1.0) * windowHalfLength).matrix());
    polygon.addVertex(center + (Eigen::Array2d(1.0, 1.0) * windowHalfLength).matrix());
    polygon.addVertex(center + (Eigen::Array2d(1.0, -1.0) * windowHalfLength).matrix());
    polygon.setFrameId(map_.getFrameId());
    geometry_msgs::msg::PolygonStamped message;
    grid_map::PolygonRosConverter::toMessage(polygon, message);
    polygonPublisher_->publish(message);

    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
}

void IteratorsDemo::publish()
{
  map_.setTimestamp(this->now().nanoseconds());
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(map_);
  RCLCPP_DEBUG(
    this->get_logger(), "Publishing grid map (timestamp %f).",
    rclcpp::Time(message->header.stamp).nanoseconds() * 1e-9);
  gridMapPublisher_->publish(std::move(message));
}

}  // namespace grid_map_demos
