/*
 * PointCloudVisualization.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__POINTCLOUDVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__POINTCLOUDVISUALIZATION_HPP_

#include <grid_map_core/GridMap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <string>

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization
{

/*!
 * Visualization of the grid map as a point cloud.
 */
class PointCloudVisualization : public VisualizationBase
{
public:
  /*!
   * Constructor.
   * @param name the name of the visualization.
   */
  explicit PointCloudVisualization(const std::string & name, rclcpp::Node::SharedPtr nodePtr);

  /*!
   * Destructor.
   */
  virtual ~PointCloudVisualization();

  /*!
   * Read parameters from ROS.
   * @param config the parameters as XML.
   * @return true if successful.
   */
  bool readParameters() override;

  /*!
   * Initialization.
   */
  bool initialize() override;

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap & map) override;

private:
  //! Type that is transformed to points.
  std::string layer_;

  //! ROS publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__POINTCLOUDVISUALIZATION_HPP_
