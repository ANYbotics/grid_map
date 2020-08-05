/*
 * GridMapRosConverter.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_ROS__GRIDMAPROSCONVERTER_HPP_
#define GRID_MAP_ROS__GRIDMAPROSCONVERTER_HPP_

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

// Eigen
#include <Eigen/Core>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <cv_bridge/cv_bridge.h>

// STL
#include <vector>
#include <unordered_map>
#include <string>
#include <memory>

namespace grid_map
{

/*!
 * ROS interface for the Grid Map library.
 */
class GridMapRosConverter
{
public:
  /*!
   * Default constructor.
   */
  GridMapRosConverter();

  /*!
   * Destructor.
   */
  virtual ~GridMapRosConverter();

  /*!
   * Converts a ROS grid map message to a grid map object.
   * @param[in] message the grid map message.
   * @param[in] layers the layers to be copied.
   * @param[in] copyBasicLayers if true, basic layers are copied.
   * @param[in] copyAllNonBasicLayers if true, all basic layers are copied, otherwise only that one specified in layers.
   * @param[out] gridMap the grid map object to be initialized.
   * @return true if successful, false otherwise.
   */
  static bool fromMessage(
    const grid_map_msgs::msg::GridMap & message, grid_map::GridMap & gridMap,
    const std::vector<std::string> & layers, bool copyBasicLayers = true,
    bool copyAllNonBasicLayers = true);

  /*!
   * Converts a ROS grid map message to a grid map object.
   * @param[in] message the grid map message.
   * @param[out] gridMap the grid map object to be initialized.
   * @return true if successful, false otherwise.
   */
  static bool fromMessage(const grid_map_msgs::msg::GridMap & message, grid_map::GridMap & gridMap);

  /*!
   * Converts all layers of a grid map object to a ROS grid map message.
   * @param[in] gridMap the grid map object.
   * @param[out] message the grid map message to be populated.
   */
  static std::unique_ptr<grid_map_msgs::msg::GridMap> toMessage(
    const grid_map::GridMap & gridMap);

  /*!
   * Converts requested layers of a grid map object to a ROS grid map message.
   * @param[in] gridMap the grid map object.
   * @param[in] layers the layers to be added to the message.
   * @param[out] message the grid map message to be populated.
   */
  static std::unique_ptr<grid_map_msgs::msg::GridMap> toMessage(
    const grid_map::GridMap & gridMap, const std::vector<std::string> & layers);

  /*!
   * Converts a grid map object to a ROS PointCloud2 message. Set the layer to be transformed
   * as the points of the point cloud with `pointLayer`, all other types will be added as
   * additional fields.
   * @param[in] gridMap the grid map object.
   * @param[in] pointLayer the type that is transformed to points.
   * @param[out] pointCloud the message to be populated.
   */
  static void toPointCloud(
    const grid_map::GridMap & gridMap,
    const std::string & pointLayer,
    sensor_msgs::msg::PointCloud2 & pointCloud);

  /*!
   * Converts a grid map object to a ROS PointCloud2 message. Set the layer to be transformed
   * as the points of the point cloud with `pointLayer` and all other types to be added as
   * additional layers with `layers`.
   * @param[in] gridMap the grid map object.
   * @param[in] layers the layers that should be added as fields to the point cloud. Must include the `pointLayer`.
   * @param[in] pointLayer the layer that is transformed to points.
   * @param[out] pointCloud the message to be populated.
   */
  static void toPointCloud(
    const grid_map::GridMap & gridMap,
    const std::vector<std::string> & layers,
    const std::string & pointLayer,
    sensor_msgs::msg::PointCloud2 & pointCloud);

  /*!
   * Converts an occupancy grid message to a layer of a grid map.
   * @param[in] occupancyGrid the occupancy grid to be converted.
   * @param[in] layer the layer to which the occupancy grid will be converted.
   * @param[out] gridMap the grid map to be populated.
   * @return true if successful.
   */
  static bool fromOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid & occupancyGrid,
    const std::string & layer, grid_map::GridMap & gridMap);

  /*!
   * Converts a grid map object to a ROS OccupancyGrid message. Set the layer to be transformed
   * as the cell data of the occupancy grid with `layer`, all other layers will be neglected.
   * @param[in] gridMap the grid map object.
   * @param[in] layer the layer that is transformed to the occupancy cell data.
   * @param[in] dataMin the minimum value of the grid map data (used to normalize the cell data in [min, max]).
   * @param[in] dataMax the maximum value of the grid map data (used to normalize the cell data in [min, max]).
   * @param[out] occupancyGrid the message to be populated.
   */
  static void toOccupancyGrid(
    const grid_map::GridMap & gridMap, const std::string & layer,
    float dataMin, float dataMax, nav_msgs::msg::OccupancyGrid & occupancyGrid);

  /*!
   * Converts an Costmap message to a layer of a grid map.
   * @param[in] Costmap the occupancy grid to be converted.
   * @param[in] layer the layer to which the costmap will be converted.
   * @param[out] gridMap the grid map to be populated.
   * @return true if successful.
   */
  static bool fromCostmap(
    const nav2_msgs::msg::Costmap & costmap,
    const std::string & layer, grid_map::GridMap & gridMap);

  /*!
   * Converts a grid map object to a ROS Costmap message. Set the layer to be transformed
   * as the cell data of the occupancy grid with `layer`, all other layers will be neglected.
   * @param[in] gridMap the grid map object.
   * @param[in] layer the layer that is transformed to the occupancy cell data.
   * @param[in] dataMin the minimum value of the grid map data (used to normalize the cell data in [min, max]).
   * @param[in] dataMax the maximum value of the grid map data (used to normalize the cell data in [min, max]).
   * @param[out] Costmap the message to be populated.
   */
  static void toCostmap(
    const grid_map::GridMap & gridMap, const std::string & layer,
    float dataMin, float dataMax, nav2_msgs::msg::Costmap & costmap);

  /*!
   * Converts a grid map object to a ROS GridCells message. Set the layer to be transformed
   * as grid cells with `layer`, all other layers will be neglected. Values that are between
   * the lower and upper threshold are converted to grid cells, other data is neglected.
   * @param[in] gridMap the grid map object.
   * @param[in] layer the layer that is transformed as grid cells.
   * @param[in] lowerThreshold the lower threshold.
   * @param[in] upperThreshold the upper threshold.
   * @param[out] gridCells the message to be populated.
   */
  static void toGridCells(
    const grid_map::GridMap & gridMap, const std::string & layer,
    float lowerThreshold, float upperThreshold,
    nav_msgs::msg::GridCells & gridCells);

  /*!
   * Initializes the geometry of a grid map from an image messages. This changes
   * the geometry of the map and deletes all contents of the layers!
   * @param[in] image the image.
   * @param[in] resolution the desired resolution of the grid map [m/cell].
   * @param[out] gridMap the grid map to be initialized.
   * @param[in](optional) position the position of the grid map.
   * @return true if successful, false otherwise.
   */
  static bool initializeFromImage(
    const sensor_msgs::msg::Image & image, const double resolution,
    grid_map::GridMap & gridMap,
    const grid_map::Position & position = grid_map::Position::Zero());

  /*!
   * Adds a layer with data from image.
   * @param[in] image the image to be added. If it is a color image
   * (bgr or bgra encoding), it will be transformed in a grayscale image.
   * @param[in] layer the layer that is filled with the image data.
   * @param[out] gridMap the grid map to be populated.
   * @param[in](optional) lowerValue value of the layer corresponding to black image pixels.
   * @param[in](optional) upperValue value of the layer corresponding to white image pixels.
   * @param[in](optional) alphaThreshold the threshold ([0.0, 1.0]) for the alpha value at which
   * cells in the grid map are marked as empty.
   * @return true if successful, false otherwise.
   */
  static bool addLayerFromImage(
    const sensor_msgs::msg::Image & image, const std::string & layer,
    grid_map::GridMap & gridMap, const float lowerValue = 0.0,
    const float upperValue = 1.0, const double alphaThreshold = 0.5);

  /*!
   * Adds a color layer with data from an image.
   * @param[in] image the image to be added.
   * @param[in] layer the layer that is filled with the image.
   * @param[out] gridMap the grid map to be populated.
   * @return true if successful, false otherwise.
   */
  static bool addColorLayerFromImage(
    const sensor_msgs::msg::Image & image, const std::string & layer,
    grid_map::GridMap & gridMap);

  /*!
   * Creates an image message from a grid map layer.
   * This conversion sets the corresponding black and white pixel value to the
   * min. and max. data of the layer data.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[out] image the message to be populated.
   * @return true if successful, false otherwise.
   */
  static bool toImage(
    const grid_map::GridMap & gridMap, const std::string & layer,
    const std::string encoding, sensor_msgs::msg::Image & image);

  /*!
   * Creates an image message from a grid map layer.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[in] lowerValue the value of the layer corresponding to black image pixels.
   * @param[in] upperValue the value of the layer corresponding to white image pixels.
   * @param[out] image the message to be populated.
   * @return true if successful, false otherwise.
   */
  static bool toImage(
    const grid_map::GridMap & gridMap, const std::string & layer,
    const std::string encoding, const float lowerValue, const float upperValue,
    sensor_msgs::msg::Image & image);

  /*!
   * Creates a cv image from a grid map layer.
   * This conversion sets the corresponding black and white pixel value to the
   * min. and max. data of the layer data.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[out] cvImage the to be populated.
   * @return true if successful, false otherwise.
   */
  static bool toCvImage(
    const grid_map::GridMap & gridMap, const std::string & layer,
    const std::string encoding, cv_bridge::CvImage & cvImage);

/*!
 * Creates a cv image from a grid map layer.
 * @param[in] grid map to be added.
 * @param[in] layer the layer that is converted to the image.
 * @param[in] encoding the desired encoding of the image.
 * @param[in] lowerValue the value of the layer corresponding to black image pixels.
 * @param[in] upperValue the value of the layer corresponding to white image pixels.
 * @param[out] cvImage to be populated.
 * @return true if successful, false otherwise.
 */
  static bool toCvImage(
    const grid_map::GridMap & gridMap, const std::string & layer,
    const std::string encoding, const float lowerValue,
    const float upperValue, cv_bridge::CvImage & cvImage);

  /*!
   * Saves a grid map into a ROS bag. The timestamp of the grid map
   * is used as time for storing the message in the ROS bag. The time
   * value 0.0 is not a valid bag time and will be replaced by the
   * current time.
   * @param[in] gridMap the grid map object to be saved in the ROS bag.
   * @param[in] pathToBag the path to the ROS bag file.
   * @param[in] topic the name of the topic in the ROS bag.
   * @return true if successful, false otherwise.
   */
  static bool saveToBag(
    const grid_map::GridMap & gridMap, const std::string & pathToBag,
    const std::string & topic);

  /*!
   * Loads a GridMap from a ROS bag.
   * @param[in] pathToBag the path to the ROS bag file.
   * @param[in] topic the topic name of the grid map in the ROS bag.
   * @param[out] gridMap the grid map object to be initialized.
   * @return true if successful, false otherwise.
   */
  static bool loadFromBag(
    const std::string & pathToBag, const std::string & topic,
    grid_map::GridMap & gridMap);
};

}  // namespace grid_map
#endif  // GRID_MAP_ROS__GRIDMAPROSCONVERTER_HPP_
