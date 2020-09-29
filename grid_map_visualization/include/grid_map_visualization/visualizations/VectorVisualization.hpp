/*
 * VectorVisualization.hpp
 *
 *  Created on: Sep 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__VECTORVISUALIZATION_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__VECTORVISUALIZATION_HPP_

#include <grid_map_core/GridMap.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// STD
#include <string>
#include <vector>

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization
{

/*!
 * Visualization a combination of three layers of the grid map as a vector field.
 */
class VectorVisualization : public VisualizationBase
{
public:
  /*!
   * Constructor.
   * @param name the name of the visualization.
   */
  explicit VectorVisualization(const std::string & name, rclcpp::Node::SharedPtr nodePtr);

  /*!
   * Destructor.
   */
  virtual ~VectorVisualization();

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
  //! Marker to be published.
  visualization_msgs::msg::Marker marker_;

  //! Types that are transformed to vectors.
  std::vector<std::string> types_;

  //! Type that is the position of the vectors.
  std::string positionLayer_;

  //! Scaling of the vectors.
  double scale_;

  //! Width of the line markers [m].
  double lineWidth_;

  //! Color of the vectors.
  std_msgs::msg::ColorRGBA color_;

  //! ROS publisher.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__VECTORVISUALIZATION_HPP_
