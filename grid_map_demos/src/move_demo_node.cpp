#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;
using namespace ros;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  init(argc, argv, "move_demo");
  NodeHandle nh("~");
  Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  GridMap map({"layer"});
  map.setFrameId("map");
  map.setGeometry(Length(0.7, 0.7), 0.01, Position(0.0, 0.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());
  map["layer"].setRandom();

  bool useMoveMethod = true;
  while (nh.ok()) {

    if (useMoveMethod) {
      ROS_INFO("Using the `move(...)` method.");
    } else {
      ROS_INFO("Using the `setPosition(...)` method.");
    }

    // Work with temporary map in a loop.
    GridMap tempMap(map);
    Rate rate(10.0);
    ros::Time startTime = ros::Time::now();
    ros::Duration duration(0.0);

    while (duration <= ros::Duration(10.0)) {
      ros::Time time = ros::Time::now();
      duration = time - startTime;

      // Change position of the map with either the `move` or `setPosition` method.
      const double t = duration.toSec();
      Position newPosition = 0.03 * t * Position(cos(t), sin(t));

      if (useMoveMethod) {
        tempMap.move(newPosition);
      } else {
        tempMap.setPosition(newPosition);
      }

      // Publish grid map.
      tempMap.setTimestamp(time.toNSec());
      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(tempMap, message);
      publisher.publish(message);
      ROS_DEBUG("Grid map (duration %f) published with new position [%f, %f].",
                duration.toSec(), tempMap.getPosition().x(), tempMap.getPosition().y());
      rate.sleep();
    }

    useMoveMethod = !useMoveMethod;
  }

  return 0;
}
