#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("move_demo");

  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(
      1).transient_local());

  // Create grid map.
  grid_map::GridMap map({"layer"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(0.7, 0.7), 0.01, grid_map::Position(0.0, 0.0));
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).\n"
    " The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());
  map["layer"].setRandom();

  bool useMoveMethod = true;
  while (rclcpp::ok()) {
    if (useMoveMethod) {
      RCLCPP_INFO(node->get_logger(), "Using the `move(...)` method.");
    } else {
      RCLCPP_INFO(node->get_logger(), "Using the `setPosition(...)` method.");
    }

    // Work with temporary map in a loop.
    grid_map::GridMap tempMap(map);
    rclcpp::Rate rate(10.0);
    rclcpp::Time startTime = node->now();
    rclcpp::Duration duration(0.0ns);

    while (duration <= rclcpp::Duration::from_seconds(10.0)) {
      rclcpp::Time time = node->now();
      duration = time - startTime;

      // Change position of the map with either the `move` or `setPosition` method.
      const double t = duration.seconds();
      grid_map::Position newPosition = 0.03 * t * grid_map::Position(cos(t), sin(t));

      if (useMoveMethod) {
        tempMap.move(newPosition);
      } else {
        tempMap.setPosition(newPosition);
      }

      // Publish grid map.
      tempMap.setTimestamp(time.nanoseconds());
      auto message = grid_map::GridMapRosConverter::toMessage(tempMap);
      publisher->publish(std::move(message));
      RCLCPP_DEBUG(
        node->get_logger(),
        "Grid map (duration %f) published with new position [%f, %f].",
        duration.seconds(), tempMap.getPosition().x(), tempMap.getPosition().y());
      rate.sleep();
    }

    useMoveMethod = !useMoveMethod;
  }

  return 0;
}
