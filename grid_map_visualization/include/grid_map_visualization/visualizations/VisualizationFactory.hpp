/*
 * VisualizationFactory.hpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_VISUALIZATION__VISUALIZATIONS__VISUALIZATIONFACTORY_HPP_
#define GRID_MAP_VISUALIZATION__VISUALIZATIONS__VISUALIZATIONFACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

namespace grid_map_visualization
{

class VisualizationFactory
{
public:
  explicit VisualizationFactory(rclcpp::Node::SharedPtr nodePtr);
  virtual ~VisualizationFactory();

  bool isValidType(const std::string & type);

  std::shared_ptr<VisualizationBase> getInstance(
    const std::string & type,
    const std::string & name);

private:
  std::vector<std::string> types_;
  rclcpp::Node::SharedPtr nodePtr_;
};

}  // namespace grid_map_visualization
#endif  // GRID_MAP_VISUALIZATION__VISUALIZATIONS__VISUALIZATIONFACTORY_HPP_
