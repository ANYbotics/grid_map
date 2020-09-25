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

VisualizationBase::VisualizationBase(const std::string & name, rclcpp::Node::SharedPtr nodePtr)
: name_(name),
  nodePtr_(nodePtr)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::isActive() const
{
  return static_cast<bool>(nodePtr_->count_subscribers(name_));
}
}  // namespace grid_map_visualization
