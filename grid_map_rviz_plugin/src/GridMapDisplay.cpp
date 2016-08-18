/*
 * GridMapDisplay.cpp
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi
 *  Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>

#include "grid_map_rviz_plugin/modified/frame_manager.h"
#include "grid_map_rviz_plugin/GridMapVisual.hpp"
#include "grid_map_rviz_plugin/GridMapDisplay.hpp"

namespace grid_map_rviz_plugin
{
  GridMapDisplay::GridMapDisplay()
  {
    alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
					      "0 is fully transparent, 1.0 is fully opaque.",
					      this, SLOT(updateVisualization()));

    history_length_property_ = new rviz::IntProperty("History Length", 1,
						     "Number of prior grid maps to display.",
						     this, SLOT(updateHistoryLength()));

    show_grid_lines_property_ = new rviz::BoolProperty("Show Grid Lines", true,
						       "Whether to show the lines connecting the grid cells.",
						       this, SLOT(updateVisualization()));

    height_mode_property_ = new rviz::EnumProperty("Height Transformer", "GridMapLayer",
						   "Select the transformer to use to set the height.",
						   this, SLOT(updateHeightMode()));
    height_mode_property_->addOption("GridMapLayer", 0);
    height_mode_property_->addOption("FlatTerrain", 1);

    height_transformer_property_ = new rviz::EditableEnumProperty("Height Layer", "elevation",
								  "Select the grid map layer to compute the height.",
								  this, SLOT(updateVisualization()));

    color_mode_property_ = new rviz::EnumProperty("Color Transformer", "GridMapLayer",
						  "Select the transformer to use to set the color.",
						  this, SLOT(updateColorMode()));
    color_mode_property_->addOption("GridMapLayer", 0);
    color_mode_property_->addOption("FlatColor", 1);

    color_transformer_property_ = new rviz::EditableEnumProperty("Color Layer", "elevation",
								 "Select the grid map layer to compute the color.",
								 this, SLOT(updateVisualization()));

    color_property_ = new rviz::ColorProperty("Color", QColor(200, 200, 200),
					      "Color to draw the mesh.",
					      this, SLOT(updateVisualization()));
    color_property_->hide();

    use_rainbow_property_ = new rviz::BoolProperty("Use Rainbow", true,
						   "Whether to use a rainbow of colors or to interpolate between two colors.",
						   this, SLOT(updateUseRainbow()));

    min_color_property_ = new rviz::ColorProperty("Min Color", QColor(0, 0, 0),
						  "Color to assign to cells with the minimum intensity.  "
						  "Actual color is interpolated between this and Max Color.",
						  this, SLOT(updateVisualization()));
    min_color_property_->hide();

    max_color_property_ = new rviz::ColorProperty("Max Color", QColor(255, 255, 255),
						  "Color to assign to cells with the maximum intensity.  "
						  "Actual color is interpolated between Min Color and this.",
						  this, SLOT(updateVisualization()));
    max_color_property_->hide();

    autocompute_intensity_bounds_property_ = new BoolProperty("Autocompute Intensity Bounds", true,
                                                              "Whether to automatically compute the intensity min/max values.",
                                                              this, SLOT(updateAutocomputeIntensityBounds()));

    min_intensity_property_ = new rviz::FloatProperty("Min Intensity", 0.0,
						      "Minimum possible intensity value, used to interpolate from Min Color to Max Color.", this, SLOT(updateVisualization()));
    min_intensity_property_->hide();

    max_intensity_property_ = new rviz::FloatProperty("Max Intensity", 10.0,
						      "Maximum possible intensity value, used to interpolate from Min Color to Max Color.", this, SLOT(updateVisualization()));
    max_intensity_property_->hide();

    history_length_property_->setMin(1);
    history_length_property_->setMax(100);
  }

  GridMapDisplay::~GridMapDisplay()
  {
  }

  void GridMapDisplay::onInitialize()
  {
    MFDClass::onInitialize();	 //  "MFDClass" = typedef of "MessageFilterDisplay<message type>"
    updateHistoryLength();
  }

  void GridMapDisplay::reset()
  {
    MFDClass::reset();
    visuals_.clear();
  }

  void GridMapDisplay::updateHistoryLength()
  {
    visuals_.rset_capacity(history_length_property_->getInt());
  }

  void GridMapDisplay::updateHeightMode()
  {
    updateVisualization();
    height_transformer_property_->setHidden(height_mode_property_->getOptionInt() == 1);
  }

  void GridMapDisplay::updateColorMode()
  {
    updateVisualization();

    bool flat_color = color_mode_property_->getOptionInt() == 1;
    color_property_->setHidden(!flat_color);
    color_transformer_property_->setHidden(flat_color);
    use_rainbow_property_->setHidden(flat_color);
    autocompute_intensity_bounds_property_->setHidden(flat_color);
    bool use_rainbow = use_rainbow_property_->getBool();
    min_color_property_->setHidden(flat_color || (!flat_color && use_rainbow));
    max_color_property_->setHidden(flat_color || (!flat_color && use_rainbow));
    bool autocompute_intensity = autocompute_intensity_bounds_property_->getBool();
    min_intensity_property_->setHidden(flat_color || (!flat_color && autocompute_intensity));
    min_intensity_property_->setHidden(flat_color || (!flat_color && autocompute_intensity));
  }

  void GridMapDisplay::updateUseRainbow()
  {
    updateVisualization();

    min_color_property_->setHidden(use_rainbow_property_->getBool());
    max_color_property_->setHidden(use_rainbow_property_->getBool());
  }

  void GridMapDisplay::updateAutocomputeIntensityBounds()
  {
    updateVisualization();

    min_intensity_property_->setHidden(autocompute_intensity_bounds_property_->getBool());
    max_intensity_property_->setHidden(autocompute_intensity_bounds_property_->getBool());
  }

  void GridMapDisplay::updateVisualization()
  {
    float alpha = alpha_property_->getFloat();
    bool show_grid_lines = show_grid_lines_property_->getBool();
    bool flat_terrain = height_mode_property_->getOptionInt() == 1;
    std::string height_layer = height_transformer_property_->getStdString();
    bool flat_color = color_mode_property_->getOptionInt() == 1;
    Ogre::ColourValue mesh_color = color_property_->getOgreColor();
    std::string color_layer = color_transformer_property_->getStdString();
    bool use_rainbow = use_rainbow_property_->getBool();
    Ogre::ColourValue min_color = min_color_property_->getOgreColor();
    Ogre::ColourValue max_color = max_color_property_->getOgreColor();
    bool autocompute_intensity = autocompute_intensity_bounds_property_->getBool();
    float min_intensity = min_intensity_property_->getFloat();
    float max_intensity = max_intensity_property_->getFloat();

    for(size_t i = 0; i < visuals_.size(); i++)
    {
      visuals_[i]->computeVisualization(alpha, show_grid_lines, flat_terrain, height_layer,
					flat_color, mesh_color, color_layer, use_rainbow,
					min_color, max_color, autocompute_intensity,
					min_intensity, max_intensity);
    }
  }

  void GridMapDisplay::processMessage(const grid_map_msgs::GridMap::ConstPtr& msg)
  {
    // Check if transform between the message's frame and the fixed frame exists
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if(!context_->getFrameManager()->getTransform(msg->info.header.frame_id,
						  msg->info.header.stamp,
						  position, orientation))
    {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
		msg->info.header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    boost::shared_ptr<GridMapVisual> visual;
    if(visuals_.full())
    {
      visual = visuals_.front();
    }
    else
    {
      visual.reset(new GridMapVisual(context_->getSceneManager(), scene_node_));
    }

    visual->setMessage(msg);
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);

    visual->computeVisualization(alpha_property_->getFloat(),
				 show_grid_lines_property_->getBool(),
				 height_mode_property_->getOptionInt() == 1,
				 height_transformer_property_->getStdString(),
				 color_mode_property_->getOptionInt() == 1,
				 color_property_->getOgreColor(),
				 color_transformer_property_->getStdString(),
				 use_rainbow_property_->getBool(),
				 min_color_property_->getOgreColor(),
				 max_color_property_->getOgreColor(),
				 autocompute_intensity_bounds_property_->getBool(),
				 min_intensity_property_->getFloat(),
				 max_intensity_property_->getFloat());

    std::vector<std::string> layer_names = visual->getLayerNames();
    height_transformer_property_->clearOptions();
    color_transformer_property_->clearOptions();
    for (size_t i = 0; i < layer_names.size(); i++)
    {
      height_transformer_property_->addOptionStd(layer_names[i]);
      color_transformer_property_->addOptionStd(layer_names[i]);
    }

    visuals_.push_back(visual);
  }

} // end namespace grid_map_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grid_map_rviz_plugin::GridMapDisplay,rviz::Display )
