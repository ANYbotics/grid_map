/*
 * GridMapDisplay.h
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi, Péter Fankhauser
 *  Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_RVIZ_PLUGIN__GRIDMAPDISPLAY_HPP_
#define GRID_MAP_RVIZ_PLUGIN__GRIDMAPDISPLAY_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>

#include <boost/circular_buffer.hpp>

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class EditableEnumProperty;
}  // namespace properties
}  // namespace rviz_common

namespace grid_map_rviz_plugin
{

class GridMapVisual;
class GridMapDisplay : public rviz_common::MessageFilterDisplay<grid_map_msgs::msg::GridMap>
{
  Q_OBJECT

public:
  GridMapDisplay();
  virtual ~GridMapDisplay();

protected:
  void onInitialize() override;

  void reset() override;

private Q_SLOTS:
  void updateHistoryLength();
  void updateHeightMode();
  void updateColorMode();
  void updateUseRainbow();
  void updateAutocomputeIntensityBounds();
  void updateVisualization();

private:
  // Callback for incoming ROS messages
  void processMessage(grid_map_msgs::msg::GridMap::ConstSharedPtr msg) override;

  // Circular buffer for visuals
  boost::circular_buffer<boost::shared_ptr<GridMapVisual>> visuals_;

  // Property variables
  rviz_common::properties::FloatProperty * alphaProperty_;
  rviz_common::properties::IntProperty * historyLengthProperty_;
  rviz_common::properties::BoolProperty * showGridLinesProperty_;
  rviz_common::properties::EnumProperty * heightModeProperty_;
  rviz_common::properties::EditableEnumProperty * heightTransformerProperty_;
  rviz_common::properties::EnumProperty * colorModeProperty_;
  rviz_common::properties::EditableEnumProperty * colorTransformerProperty_;
  rviz_common::properties::ColorProperty * colorProperty_;
  rviz_common::properties::BoolProperty * useRainbowProperty_;
  rviz_common::properties::BoolProperty * invertRainbowProperty_;
  rviz_common::properties::ColorProperty * minColorProperty_;
  rviz_common::properties::ColorProperty * maxColorProperty_;
  rviz_common::properties::BoolProperty * autocomputeIntensityBoundsProperty_;
  rviz_common::properties::FloatProperty * minIntensityProperty_;
  rviz_common::properties::FloatProperty * maxIntensityProperty_;
};

}  // namespace grid_map_rviz_plugin
#endif  // GRID_MAP_RVIZ_PLUGIN__GRIDMAPDISPLAY_HPP_
