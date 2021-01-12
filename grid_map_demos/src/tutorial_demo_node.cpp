#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("grid_map_tutorial_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  // Create grid map.
  grid_map::GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), 0.03, grid_map::Position(0.0, -0.1));
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).\n"
    " The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

  // Work with grid map in a loop.
  rclcpp::Rate rate(30.0);
  rclcpp::Clock clock;
  while (rclcpp::ok()) {
    rclcpp::Time time = node->now();

    // Add elevation and surface normal (iterating through grid map and adding data).
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      map.at(
        "elevation",
        *it) = -0.04 + 0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()) * position.x();
      Eigen::Vector3d normal(-0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()),
        -position.x() * std::cos(3.0 * time.seconds() + 5.0 * position.y()), 1.0);
      normal.normalize();
      map.at("normal_x", *it) = normal.x();
      map.at("normal_y", *it) = normal.y();
      map.at("normal_z", *it) = normal.z();
    }

    // Add noise (using Eigen operators).
    map.add("noise", 0.015 * grid_map::Matrix::Random(map.getSize()(0), map.getSize()(1)));
    map.add("elevation_noisy", map.get("elevation") + map["noise"]);

    // Adding outliers (accessing cell by position).
    for (unsigned int i = 0; i < 500; ++i) {
      grid_map::Position randomPosition = grid_map::Position::Random();
      if (map.isInside(randomPosition)) {
        map.atPosition("elevation_noisy", randomPosition) = std::numeric_limits<float>::infinity();
      }
    }

    // Filter values for submap (iterators).
    map.add("elevation_filtered", map.get("elevation_noisy"));
    grid_map::Position topLeftCorner(1.0, 0.4);
    grid_map::boundPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
    grid_map::Index startIndex;
    map.getIndex(topLeftCorner, startIndex);
    RCLCPP_INFO_ONCE(
      node->get_logger(),
      "Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
      topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

    grid_map::Size size = (grid_map::Length(1.2, 0.8) / map.getResolution()).cast<int>();
    grid_map::SubmapIterator it(map, startIndex, size);
    for (; !it.isPastEnd(); ++it) {
      grid_map::Position currentPosition;
      map.getPosition(*it, currentPosition);
      double radius = 0.1;
      double mean = 0.0;
      double sumOfWeights = 0.0;

      // Compute weighted mean.
      for (grid_map::CircleIterator circleIt(map, currentPosition, radius); !circleIt.isPastEnd();
        ++circleIt)
      {
        if (!map.isValid(*circleIt, "elevation_noisy")) {continue;}
        grid_map::Position currentPositionInCircle;
        map.getPosition(*circleIt, currentPositionInCircle);

        // Computed weighted mean based on Euclidian distance.
        double distance = (currentPosition - currentPositionInCircle).norm();
        double weight = pow(radius - distance, 2);
        mean += weight * map.at("elevation_noisy", *circleIt);
        sumOfWeights += weight;
      }

      map.at("elevation_filtered", *it) = mean / sumOfWeights;
    }

    // Show absolute difference and compute mean squared error.
    map.add("error", (map.get("elevation_filtered") - map.get("elevation")).cwiseAbs());

    // Publish grid map.
    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);
    publisher->publish(std::move(message));
    RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 1000, "Grid map published.");

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

  return 0;
}
