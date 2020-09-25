/*
 * VisualizationFactory.cpp
 *
 *  Created on: Mar 20, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

// STL
#include <algorithm>
#include <memory>
#include <string>

#include "grid_map_visualization/visualizations/VisualizationFactory.hpp"
#include "grid_map_visualization/visualizations/PointCloudVisualization.hpp"
#include "grid_map_visualization/visualizations/FlatPointCloudVisualization.hpp"
#include "grid_map_visualization/visualizations/VectorVisualization.hpp"
#include "grid_map_visualization/visualizations/OccupancyGridVisualization.hpp"
#include "grid_map_visualization/visualizations/GridCellsVisualization.hpp"
#include "grid_map_visualization/visualizations/MapRegionVisualization.hpp"

namespace grid_map_visualization
{

VisualizationFactory::VisualizationFactory(rclcpp::Node::SharedPtr nodePtr)
: nodePtr_(nodePtr)
{
  types_.push_back("point_cloud");
  types_.push_back("flat_point_cloud");
  types_.push_back("vectors");
  types_.push_back("occupancy_grid");
  types_.push_back("grid_cells");
  types_.push_back("map_region");
}

VisualizationFactory::~VisualizationFactory()
{
}

bool VisualizationFactory::isValidType(const std::string & type)
{
  return end(types_) != std::find(begin(types_), end(types_), type);
}

std::shared_ptr<VisualizationBase> VisualizationFactory::getInstance(
  const std::string & type,
  const std::string & name)
{
  // TODO(needs_assignment):
  // Make this nicer: http://stackoverflow.com/questions/9975672/c-automatic-factory-registration-of-derived-types
  if (type == "point_cloud") {
    return std::shared_ptr<VisualizationBase>(new PointCloudVisualization(name, nodePtr_));
  }
  if (type == "flat_point_cloud") {
    return std::shared_ptr<VisualizationBase>(new FlatPointCloudVisualization(name, nodePtr_));
  }
  if (type == "vectors") {
    return std::shared_ptr<VisualizationBase>(new VectorVisualization(name, nodePtr_));
  }
  if (type == "occupancy_grid") {
    return std::shared_ptr<VisualizationBase>(new OccupancyGridVisualization(name, nodePtr_));
  }
  if (type == "grid_cells") {
    return std::shared_ptr<VisualizationBase>(new GridCellsVisualization(name, nodePtr_));
  }
  if (type == "map_region") {
    return std::shared_ptr<VisualizationBase>(new MapRegionVisualization(name, nodePtr_));
  }
  return std::shared_ptr<VisualizationBase>();
}

}  // namespace grid_map_visualization
