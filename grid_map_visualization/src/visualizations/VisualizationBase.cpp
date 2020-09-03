/*
 * VisualizationBase.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_visualization/visualizations/VisualizationBase.hpp"

#include <string>

namespace grid_map_visualization
{

VisualizationBase::VisualizationBase(rclcpp::Node::SharedPtr node, const std::string & name)
: node_(node),
  name_(name)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::isActive() const
{
  return static_cast<bool>(node_->count_subscribers(name_));
}
}  // namespace grid_map_visualization
