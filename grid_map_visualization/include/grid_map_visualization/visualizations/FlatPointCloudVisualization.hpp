/*
 * FlatPointCloudVisualization.hpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__FLATPOINTCLOUDVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__FLATPOINTCLOUDVISUALIZATION_HPP_

#include <grid_map_core/GridMap.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <string>

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization
{

/*!
 * Visualization of the grid map as a flat point cloud.
 */
class FlatPointCloudVisualization : public VisualizationBase
{
public:
  /*!
   * Constructor.
   * @param name the name of the visualization.
   */
  explicit FlatPointCloudVisualization(const std::string & name, rclcpp::Node::SharedPtr nodePtr);

  /*!
   * Destructor.
   */
  virtual ~FlatPointCloudVisualization();

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

  //! Height of the z-coordinate at which the flat point cloud is visualized.
  double height_;

  //! ROS publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__FLATPOINTCLOUDVISUALIZATION_HPP_
