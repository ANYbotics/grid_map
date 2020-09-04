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

VisualizationBase::VisualizationBase(const std::string & name)
: Node(name)
{
}

VisualizationBase::~VisualizationBase()
{
}

bool VisualizationBase::isActive() const
{
  return static_cast<bool>(this->count_subscribers(this->get_name()));
}
}  // namespace grid_map_visualization
