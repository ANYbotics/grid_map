#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <memory>
#include <utility>

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("resolution_change_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(
      1).transient_local());

  // Create grid map.
  grid_map::GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), 0.03);
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Add data.
  map["elevation"].setZero();  // Un/comment this line to try with and without transparency.
  grid_map::Polygon polygon;
  polygon.setFrameId(map.getFrameId());
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

  for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) {
    map.at("elevation", *iterator) = 0.3;
  }

  rclcpp::Rate rate(10.0);

  // Work in a loop.
  while (rclcpp::ok()) {
    // Initialize.
    rclcpp::Time time = node->now();
    const double resolution = 0.05 + 0.04 * sin(time.seconds());

    // Change resoltion of grid map.
    grid_map::GridMap modifiedMap;
    grid_map::GridMapCvProcessing::changeResolution(map, modifiedMap, resolution);

    // Publish grid map.
    auto message = grid_map::GridMapRosConverter::toMessage(modifiedMap);
    publisher->publish(std::move(message));
    RCLCPP_INFO_STREAM(
      node->get_logger(), "Published grid map with " << resolution << " m/cell resolution.");

    rate.sleep();
  }

  return 0;
}
