#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

using namespace grid_map;
using namespace ros;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  init(argc, argv, "resolution_change_demo");
  NodeHandle nodeHandle("~");
  Publisher publisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Add data.
  map["elevation"].setZero(); // Un/comment this line to try with and without transparency.
  grid_map::Polygon polygon;
  polygon.setFrameId(map.getFrameId());
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

  for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) {
    map.at("elevation", *iterator) = 0.3;
  }

  Rate rate(10.0);

  // Work in a loop.
  while (nodeHandle.ok()) {

    // Initialize.
    ros::Time time = ros::Time::now();
    const double resolution = 0.05 + 0.04 * sin(time.toSec());

    // Change resoltion of grid map.
    GridMap modifiedMap;
    GridMapCvProcessing::changeResolution(map, modifiedMap, resolution);

    // Publish grid map.
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(modifiedMap, message);
    publisher.publish(message);
    ROS_INFO_STREAM("Published grid map with " << resolution << " m/cell resolution.");

    rate.sleep();
  }

  return 0;
}
