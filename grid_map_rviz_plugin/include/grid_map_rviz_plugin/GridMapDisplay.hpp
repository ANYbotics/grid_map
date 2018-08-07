/*
 * GridMapDisplay.h
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi, Péter Fankhauser
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#ifndef Q_MOC_RUN
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <boost/circular_buffer.hpp>
#include "grid_map_rviz_plugin/modified/message_filter_display.h"
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class EditableEnumProperty;
}

namespace grid_map_rviz_plugin {

class GridMapVisual;
class GridMapDisplay : public MessageFilterDisplayMod<grid_map_msgs::GridMap>
{
Q_OBJECT
 public:
  GridMapDisplay();
  virtual ~GridMapDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private Q_SLOTS:
  void updateHistoryLength();
  void updateHeightMode();
  void updateColorMode();
  void updateUseRainbow();
  void updateAutocomputeIntensityBounds();
  void updateVisualization();

 private:
  // Callback for incoming ROS messages
  void processMessage(const grid_map_msgs::GridMap::ConstPtr& msg);

  // Circular buffer for visuals
  boost::circular_buffer<boost::shared_ptr<GridMapVisual> > visuals_;

  // Property variables
  rviz::FloatProperty* alphaProperty_;
  rviz::IntProperty* historyLengthProperty_;
  rviz::BoolProperty* showGridLinesProperty_;
  rviz::EnumProperty* heightModeProperty_;
  rviz::EditableEnumProperty* heightTransformerProperty_;
  rviz::EnumProperty* colorModeProperty_;
  rviz::EditableEnumProperty* colorTransformerProperty_;
  rviz::ColorProperty* colorProperty_;
  rviz::BoolProperty* useRainbowProperty_;
  rviz::BoolProperty* invertRainbowProperty_;
  rviz::ColorProperty* minColorProperty_;
  rviz::ColorProperty* maxColorProperty_;
  rviz::BoolProperty* autocomputeIntensityBoundsProperty_;
  rviz::FloatProperty* minIntensityProperty_;
  rviz::FloatProperty* maxIntensityProperty_;
};

}  // end namespace grid_map_rviz_plugin
