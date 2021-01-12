#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <utility>

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("opencv_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(
      1).transient_local());
  const bool useTransparency = false;

  // Create grid map.
  grid_map::GridMap map({"original", "elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.2, 2.0), 0.01);
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Add data.
  if (!useTransparency) {map["original"].setZero();}
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
    map.at("original", *iterator) = 0.3;
  }

  // Convert to CV image.
  cv::Mat originalImage;
  if (useTransparency) {
    // Note: The template parameters have to be set based on your encoding
    // of the image. For 8-bit images use `unsigned char`.
    grid_map::GridMapCvConverter::toImage<uint16_t, 4>(
      map, "original", CV_16UC4, 0.0, 0.3,
      originalImage);
  } else {
    grid_map::GridMapCvConverter::toImage<uint16_t, 1>(
      map, "original", CV_16UC1, 0.0, 0.3,
      originalImage);
  }

  // Create OpenCV window.
  cv::namedWindow("OpenCV Demo");

  // Work with copy of image in a loop.
  while (rclcpp::ok()) {
    // Initialize.
    rclcpp::Time time = node->now();
    map.setTimestamp(time.nanoseconds());
    cv::Mat modifiedImage;
    int blurRadius = 200 - abs(static_cast<int>(200.0 * sin(time.seconds())));
    blurRadius = blurRadius - (blurRadius % 2) + 1;

    // Apply Gaussian blur.
    cv::GaussianBlur(originalImage, modifiedImage, cv::Size(blurRadius, blurRadius), 0.0, 0.0);

    // Visualize as image.
    cv::imshow("OpenCV Demo", modifiedImage);
    cv::waitKey(40);

    // Convert resulting image to a grid map.
    if (useTransparency) {
      grid_map::GridMapCvConverter::addLayerFromImage<uint16_t, 4>(
        modifiedImage, "elevation", map, 0.0,
        0.3, 0.3);
    } else {
      grid_map::GridMapCvConverter::addLayerFromImage<uint16_t, 1>(
        modifiedImage, "elevation", map, 0.0,
        0.3);
    }

    // Publish grid map.
    auto message = grid_map::GridMapRosConverter::toMessage(map);
    publisher->publish(std::move(message));
    RCLCPP_INFO_STREAM(
      node->get_logger(), "Published image and grid map with blur radius " << blurRadius << ".");
  }

  return 0;
}
