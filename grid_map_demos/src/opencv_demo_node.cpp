#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>

using namespace grid_map;
using namespace ros;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  init(argc, argv, "opencv_demo");
  NodeHandle nh("~");
  Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  const bool useTransparency = false;

  // Create grid map.
  GridMap map({"original", "elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.01);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Add data.
  if (!useTransparency) map["original"].setZero();
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
    map.at("original", *iterator) = 0.3;
  }

  // Convert to CV image.
  cv_bridge::CvImage originalImage;
  std::string encoding;
  if (useTransparency) {
    encoding = sensor_msgs::image_encodings::RGBA16;
  } else {
    encoding = sensor_msgs::image_encodings::MONO16;
  }
  GridMapRosConverter::toCvImage(map, "original", encoding, 0.0, 0.3, originalImage);

  // Create OpenCV window.
  cv::namedWindow("OpenCV Demo");

  // Work with copy of image in a loop.
  while (nh.ok()) {

    // Initialize.
    ros::Time time = ros::Time::now();
    map.setTimestamp(time.toNSec());
    cv_bridge::CvImage modifiedImage(originalImage.header, originalImage.encoding);
    int blurRadius = 200 - abs((int)(200.0 * sin(time.toSec())));
    blurRadius = blurRadius - (blurRadius % 2) + 1;

    // Apply Gaussian blur.
    cv::GaussianBlur(originalImage.image, modifiedImage.image, cv::Size(blurRadius, blurRadius), 0.0, 0.0);

    // Visualize as image.
    cv::imshow("OpenCV Demo", modifiedImage.image);
    cv::waitKey(40);

    // Convert resulting image to a grid map.
    // Note: The template parameters have to be set based on your encoding
    // of the image. For MONO8 encoding use `unsigned char`.
    if (useTransparency) {
      GridMapRosConverter::addLayerFromImage<unsigned short, 4>(modifiedImage.image, "elevation", map, 0.0, 0.3, 0.3);
    } else {
      GridMapRosConverter::addLayerFromImage<unsigned short, 1>(modifiedImage.image, "elevation", map, 0.0, 0.3);
    }

    // Publish grid map.
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_STREAM("Published image and grid map with blur radius " << blurRadius << ".");
  }

  return 0;
}
