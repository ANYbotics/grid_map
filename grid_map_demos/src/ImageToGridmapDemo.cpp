/*
 * ImageToGridmapDemo.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <string>
#include <utility>

#include "grid_map_demos/ImageToGridmapDemo.hpp"

namespace grid_map_demos
{

ImageToGridmapDemo::ImageToGridmapDemo()
: Node("image_to_gridmap_demo"),
  map_(grid_map::GridMap({"elevation"})),
  mapInitialized_(false)
{
  readParameters();
  map_.setBasicLayers({"elevation"});
  imageSubscriber_ =
    this->create_subscription<sensor_msgs::msg::Image>(
    imageTopic_, 1,
    std::bind(&ImageToGridmapDemo::imageCallback, this, std::placeholders::_1));

  gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());
}

ImageToGridmapDemo::~ImageToGridmapDemo()
{
}

bool ImageToGridmapDemo::readParameters()
{
  this->declare_parameter("image_topic", std::string("/image"));
  this->declare_parameter("resolution", rclcpp::ParameterValue(0.03));
  this->declare_parameter("min_height", rclcpp::ParameterValue(0.0));
  this->declare_parameter("max_height", rclcpp::ParameterValue(1.0));

  this->get_parameter("image_topic", imageTopic_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("min_height", minHeight_);
  this->get_parameter("max_height", maxHeight_);
  return true;
}

void ImageToGridmapDemo::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!mapInitialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(*msg, resolution_, map_);
    RCLCPP_INFO(
      this->get_logger(),
      "Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
      map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    mapInitialized_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(*msg, "elevation", map_, minHeight_, maxHeight_);
  grid_map::GridMapRosConverter::addColorLayerFromImage(*msg, "color", map_);

  // Publish as grid map.
  auto message = grid_map::GridMapRosConverter::toMessage(map_);
  gridMapPublisher_->publish(std::move(message));
}

}  // namespace grid_map_demos
