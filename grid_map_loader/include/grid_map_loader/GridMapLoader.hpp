/*
 * GridMapLoader.hpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#ifndef GRID_MAP_LOADER__GRIDMAPLOADER_HPP_
#define GRID_MAP_LOADER__GRIDMAPLOADER_HPP_

// ROS
#include <rclcpp/rclcpp.hpp>

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>

// STD
#include <string>

namespace grid_map_loader
{

/*!
 * Loads and publishes a grid map from a bag file.
 */
class GridMapLoader : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   */
  GridMapLoader();

  /*!
   * Destructor.
   */
  virtual ~GridMapLoader();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  /*!
   * Loads the grid map from the bag file.
   * @return true if successful, false otherwise.
   */
  bool load();

  /*!
   * Publishes the grid map.
   */
  void publish();

private:
  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Path the ROS bag to be published.
  std::string filePath_;

  //! Topic name of the grid map in the ROS bag.
  std::string bagTopic_;

  //! Topic name of the grid map to be loaded.
  std::string publishTopic_;

  //! Duration to publish the grid map.
  double durationInSec;

  //! QOS Transient local state
  bool qos_transient_local_;
};

}  // namespace grid_map_loader
#endif  // GRID_MAP_LOADER__GRIDMAPLOADER_HPP_
