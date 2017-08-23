#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_tutorial_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03, Position(0.0, -0.1));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();

    // Add elevation and surface normal (iterating through grid map and adding data).
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
      Eigen::Vector3d normal(-0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()),
                             -position.x() * std::cos(3.0 * time.toSec() + 5.0 * position.y()), 1.0);
      normal.normalize();
      map.at("normal_x", *it) = normal.x();
      map.at("normal_y", *it) = normal.y();
      map.at("normal_z", *it) = normal.z();
    }

    // Add noise (using Eigen operators).
    map.add("noise", 0.015 * Matrix::Random(map.getSize()(0), map.getSize()(1)));
    map.add("elevation_noisy", map.get("elevation") + map["noise"]);

    // Adding outliers (accessing cell by position).
    for (unsigned int i = 0; i < 500; ++i) {
      Position randomPosition = Position::Random();
      if (map.isInside(randomPosition))
        map.atPosition("elevation_noisy", randomPosition) = std::numeric_limits<float>::infinity();
    }

    // Filter values for submap (iterators).
    map.add("elevation_filtered", map.get("elevation_noisy"));
    Position topLeftCorner(1.0, 0.4);
    boundPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
    Index startIndex;
    map.getIndex(topLeftCorner, startIndex);
    ROS_INFO_ONCE("Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
             topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

    Size size = (Length(1.2, 0.8) / map.getResolution()).cast<int>();
    SubmapIterator it(map, startIndex, size);
    for (; !it.isPastEnd(); ++it) {
      Position currentPosition;
      map.getPosition(*it, currentPosition);
      double radius = 0.1;
      double mean = 0.0;
      double sumOfWeights = 0.0;

      // Compute weighted mean.
      for (CircleIterator circleIt(map, currentPosition, radius);
          !circleIt.isPastEnd(); ++circleIt) {
        if (!map.isValid(*circleIt, "elevation_noisy")) continue;
        Position currentPositionInCircle;
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
    unsigned int nCells = map.getSize().prod();
    // cppcheck-suppress unreadVariable
    double rootMeanSquaredError = sqrt((map["error"].array().pow(2).sum()) / nCells);

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    rate.sleep();
  }

  return 0;
}
