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

VisualizationBase::VisualizationBase(const std::string & name, rclcpp::Node::SharedPtr node_ptr)
: name_(name),
  node_ptr_(node_ptr)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::isActive() const
{
  return static_cast<bool>(node_ptr_->count_subscribers(name_));
}
}  // namespace grid_map_visualization
