/*
 * GridMapDisplay.h
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi
 *  Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef GRID_MAP_DISPLAY_H
#define GRID_MAP_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include "message_filter_display_mod.h"
#include <sensor_msgs/Imu.h>
#endif

#include <grid_map_msgs/GridMap.h>

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
  class BoolProperty;
  class ColorProperty;
  class FloatProperty;
  class IntProperty;
  class EnumProperty;
  class EditableEnumProperty;
}

namespace grid_map_rviz_plugin
{
  class GridMapVisual;
  class GridMapDisplay: public MessageFilterDisplayMod<grid_map_msgs::GridMap, grid_map_msgs::GridMapInfo>
  {
  Q_OBJECT
  public:
    // Constructor
    GridMapDisplay();
    // Destructor
    virtual ~GridMapDisplay();

  protected:
    virtual void onInitialize();

    virtual void reset();
   
  private Q_SLOTS: 
    // Qt slots
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
    rviz::FloatProperty* alpha_property_;
    rviz::IntProperty* history_length_property_;
    rviz::BoolProperty* show_grid_lines_property_;
    rviz::EnumProperty* height_mode_property_;
    rviz::EditableEnumProperty* height_transformer_property_; 
    rviz::EnumProperty* color_mode_property_;
    rviz::EditableEnumProperty* color_transformer_property_;
    rviz::ColorProperty* color_property_;    
    rviz::BoolProperty* use_rainbow_property_;
    rviz::ColorProperty* min_color_property_;
    rviz::ColorProperty* max_color_property_;
    rviz::BoolProperty* autocompute_intensity_bounds_property_;
    rviz::FloatProperty* min_intensity_property_;
    rviz::FloatProperty* max_intensity_property_;
  };

} // end namespace grid_map_rviz_plugin

#endif // GRID_MAP_DISPLAY_H
