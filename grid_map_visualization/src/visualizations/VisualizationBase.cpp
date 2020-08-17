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

VisualizationBase::VisualizationBase(rclcpp::Node::SharedPtr nodeHandle, const std::string & name)
: nodeHandle_(nodeHandle),
  name_(name)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::isActive() const
{
  if (nodeHandle_->count_subscribers(name_) > 0) {return true;}
  return false;
}
}  // namespace grid_map_visualization
