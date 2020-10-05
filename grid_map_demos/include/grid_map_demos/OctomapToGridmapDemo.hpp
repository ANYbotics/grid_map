/*
 * OctomapToGridmapDemo.hpp
 *
 *  Created on: May 03, 2017
 *      Author: Jeff Delmerico
 *   Institute: University of Zürich, Robotics and Perception Group
 */

#ifndef GRID_MAP_DEMOS__OCTOMAPTOGRIDMAPDEMO_HPP_
#define GRID_MAP_DEMOS__OCTOMAPTOGRIDMAPDEMO_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace grid_map_demos
{

/*!
 * Receives a volumetric OctoMap and converts it to a grid map with an elevation layer.
 * The grid map is published and can be viewed in Rviz.
 */
class OctomapToGridmapDemo : public rclcpp::Node
{
public:
  using GetOctomapSrv = octomap_msgs::srv::GetOctomap;
  using OctomapMessage = octomap_msgs::msg::Octomap;

  /*!
   * Constructor.
   */
  OctomapToGridmapDemo();

  /*!
   * Destructor.
   */
  virtual ~OctomapToGridmapDemo();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void convertAndPublishMap();

private:
  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;

  //! Octomap publisher.
  rclcpp::Publisher<OctomapMessage>::SharedPtr octomapPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Name of the grid map topic.
  std::string octomapServiceTopic_;

  //! Octomap service client
  rclcpp::Client<GetOctomapSrv>::SharedPtr client_;

  //! Bounding box of octomap to convert.
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;
};

}  // namespace grid_map_demos
#endif  // GRID_MAP_DEMOS__OCTOMAPTOGRIDMAPDEMO_HPP_
