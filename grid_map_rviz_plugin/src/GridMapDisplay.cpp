/*
 * GridMapDisplay.cpp
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi, Péter Fankhauser
 *  Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_rviz_plugin/GridMapVisual.hpp"
#include "grid_map_rviz_plugin/GridMapDisplay.hpp"
#include "grid_map_rviz_plugin/modified/frame_manager.h"

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

namespace grid_map_rviz_plugin {

GridMapDisplay::GridMapDisplay()
{
  alphaProperty_ = new rviz::FloatProperty("Alpha", 1.0,
                                           "0 is fully transparent, 1.0 is fully opaque.", this,
                                           SLOT(updateVisualization()));

  historyLengthProperty_ = new rviz::IntProperty("History Length", 1,
                                                 "Number of prior grid maps to display.", this,
                                                 SLOT(updateHistoryLength()));

  showGridLinesProperty_ = new rviz::BoolProperty(
      "Show Grid Lines", true, "Whether to show the lines connecting the grid cells.", this,
      SLOT(updateVisualization()));

  heightModeProperty_ = new rviz::EnumProperty("Height Transformer", "GridMapLayer",
                                               "Select the transformer to use to set the height.",
                                               this, SLOT(updateHeightMode()));
  heightModeProperty_->addOption("Layer", 0);
  heightModeProperty_->addOption("Flat", 1);

  heightTransformerProperty_ = new rviz::EditableEnumProperty(
      "Height Layer", "elevation", "Select the grid map layer to compute the height.", this,
      SLOT(updateVisualization()));

  colorModeProperty_ = new rviz::EnumProperty("Color Transformer", "GridMapLayer",
                                              "Select the transformer to use to set the color.",
                                              this, SLOT(updateColorMode()));
  colorModeProperty_->addOption("IntensityLayer", 0);
  colorModeProperty_->addOption("ColorLayer", 1);
  colorModeProperty_->addOption("FlatColor", 2);
  colorModeProperty_->addOption("None", 3);

  colorTransformerProperty_ = new rviz::EditableEnumProperty(
      "Color Layer", "elevation", "Select the grid map layer to compute the color.", this,
      SLOT(updateVisualization()));

  colorProperty_ = new rviz::ColorProperty("Color", QColor(200, 200, 200),
                                           "Color to draw the mesh.", this,
                                           SLOT(updateVisualization()));
  colorProperty_->hide();

  useRainbowProperty_ = new rviz::BoolProperty(
      "Use Rainbow", true,
      "Whether to use a rainbow of colors or to interpolate between two colors.", this,
      SLOT(updateUseRainbow()));
  
  invertRainbowProperty_ = new rviz::BoolProperty(
      "Invert Rainbow", false,
      "Whether to invert the rainbow colors.", this,
      SLOT(updateVisualization()));

  minColorProperty_ = new rviz::ColorProperty(
      "Min Color", QColor(0, 0, 0), "Color to assign to cells with the minimum intensity.  "
      "Actual color is interpolated between this and Max Color.",
      this, SLOT(updateVisualization()));
  minColorProperty_->hide();

  maxColorProperty_ = new rviz::ColorProperty(
      "Max Color", QColor(255, 255, 255), "Color to assign to cells with the maximum intensity.  "
      "Actual color is interpolated between Min Color and this.",
      this, SLOT(updateVisualization()));
  maxColorProperty_->hide();

  autocomputeIntensityBoundsProperty_ = new BoolProperty(
      "Autocompute Intensity Bounds", true,
      "Whether to automatically compute the intensity min/max values.", this,
      SLOT(updateAutocomputeIntensityBounds()));

  minIntensityProperty_ = new rviz::FloatProperty(
      "Min Intensity", 0.0,
      "Minimum possible intensity value, used to interpolate from Min Color to Max Color.", this,
      SLOT(updateVisualization()));
  minIntensityProperty_->hide();

  maxIntensityProperty_ = new rviz::FloatProperty(
      "Max Intensity", 10.0,
      "Maximum possible intensity value, used to interpolate from Min Color to Max Color.", this,
      SLOT(updateVisualization()));
  maxIntensityProperty_->hide();

  historyLengthProperty_->setMin(1);
  historyLengthProperty_->setMax(100);
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
  visuals_.rset_capacity(historyLengthProperty_->getInt());
}

void GridMapDisplay::updateHeightMode()
{
  updateVisualization();
  heightTransformerProperty_->setHidden(heightModeProperty_->getOptionInt() == 1);
}

void GridMapDisplay::updateColorMode()
{
  updateVisualization();
  
  bool intensityColor = colorModeProperty_->getOptionInt() == 0;
  bool flatColor = colorModeProperty_->getOptionInt() == 2;
  bool none = colorModeProperty_->getOptionInt() == 3;
  colorProperty_->setHidden(!flatColor);
  colorTransformerProperty_->setHidden(flatColor || none);
  useRainbowProperty_->setHidden(!intensityColor);
  invertRainbowProperty_->setHidden(!intensityColor);
  autocomputeIntensityBoundsProperty_->setHidden(!intensityColor);
  bool useRainbow = useRainbowProperty_->getBool();
  minColorProperty_->setHidden(!intensityColor || useRainbow);
  maxColorProperty_->setHidden(!intensityColor || useRainbow);
  bool autocomputeIntensity = autocomputeIntensityBoundsProperty_->getBool();
  minIntensityProperty_->setHidden(!intensityColor || autocomputeIntensity);
  minIntensityProperty_->setHidden(!intensityColor || autocomputeIntensity);
}

void GridMapDisplay::updateUseRainbow()
{
  updateVisualization();
  bool useRainbow = useRainbowProperty_->getBool();
  minColorProperty_->setHidden(useRainbow);
  maxColorProperty_->setHidden(useRainbow);
  invertRainbowProperty_->setHidden(!useRainbow);
}

void GridMapDisplay::updateAutocomputeIntensityBounds()
{
  updateVisualization();
  minIntensityProperty_->setHidden(autocomputeIntensityBoundsProperty_->getBool());
  maxIntensityProperty_->setHidden(autocomputeIntensityBoundsProperty_->getBool());
}

void GridMapDisplay::updateVisualization()
{
  float alpha = alphaProperty_->getFloat();
  bool showGridLines = showGridLinesProperty_->getBool();
  bool flatTerrain = heightModeProperty_->getOptionInt() == 1;
  std::string heightLayer = heightTransformerProperty_->getStdString();
  bool mapLayerColor = colorModeProperty_->getOptionInt() == 1;
  bool flatColor = colorModeProperty_->getOptionInt() == 2;
  bool noColor = colorModeProperty_->getOptionInt() == 3;
  Ogre::ColourValue meshColor = colorProperty_->getOgreColor();
  std::string colorLayer = colorTransformerProperty_->getStdString();
  bool useRainbow = useRainbowProperty_->getBool();
  bool invertRainbow = invertRainbowProperty_->getBool();
  Ogre::ColourValue minColor = minColorProperty_->getOgreColor();
  Ogre::ColourValue maxColor = maxColorProperty_->getOgreColor();
  bool autocomputeIntensity = autocomputeIntensityBoundsProperty_->getBool();
  float minIntensity = minIntensityProperty_->getFloat();
  float maxIntensity = maxIntensityProperty_->getFloat();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->computeVisualization(alpha, showGridLines, flatTerrain, heightLayer, flatColor, noColor, meshColor,
                                      mapLayerColor, colorLayer, useRainbow, invertRainbow, minColor, maxColor,
                                      autocomputeIntensity, minIntensity, maxIntensity);
  }
}

void GridMapDisplay::processMessage(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  // Check if transform between the message's frame and the fixed frame exists.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->info.header.frame_id, msg->info.header.stamp,
                                                 position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->info.header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  boost::shared_ptr<GridMapVisual> visual;
  if (visuals_.full()) {
    visual = visuals_.front();
  } else {
    visual.reset(new GridMapVisual(context_->getSceneManager(), scene_node_));
  }

  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  visual->computeVisualization(alphaProperty_->getFloat(), showGridLinesProperty_->getBool(),
                               heightModeProperty_->getOptionInt() == 1, heightTransformerProperty_->getStdString(),
                               colorModeProperty_->getOptionInt() == 2, colorModeProperty_->getOptionInt() == 3,
                               colorProperty_->getOgreColor(), colorModeProperty_->getOptionInt() == 1,
                               colorTransformerProperty_->getStdString(), useRainbowProperty_->getBool(),
                               invertRainbowProperty_->getBool(), minColorProperty_->getOgreColor(),
                               maxColorProperty_->getOgreColor(), autocomputeIntensityBoundsProperty_->getBool(),
                               minIntensityProperty_->getFloat(), maxIntensityProperty_->getFloat());

  std::vector<std::string> layer_names = visual->getLayerNames();
  heightTransformerProperty_->clearOptions();
  colorTransformerProperty_->clearOptions();
  for (size_t i = 0; i < layer_names.size(); i++) {
    heightTransformerProperty_->addOptionStd(layer_names[i]);
    colorTransformerProperty_->addOptionStd(layer_names[i]);
  }

  visuals_.push_back(visual);
}

}  // end namespace grid_map_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grid_map_rviz_plugin::GridMapDisplay, rviz::Display)
