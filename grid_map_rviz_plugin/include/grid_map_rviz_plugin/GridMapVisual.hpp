/*
 * GridMapVisual.cpp
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi, Péter Fankhauser
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSharedPtr.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>

namespace Ogre {
class Vector3;
class Quaternion;
class ManualObject;
class ColourValue;
}

namespace rviz {
class BillboardLine;
}

namespace grid_map_rviz_plugin {

// Visualizes a single grid_map_msgs::GridMap message.
class GridMapVisual
{
 public:
  GridMapVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
  virtual ~GridMapVisual();

  // Copy the grid map data to map_.
  void setMessage(const grid_map_msgs::GridMap::ConstPtr& msg);
  // Compute the visualization of map_.
  void computeVisualization(float alpha, bool showGridLines, bool flatTerrain, std::string heightLayer, bool flatColor,
                            bool noColor, Ogre::ColourValue meshColor, bool mapLayerColor, std::string colorLayer,
                            bool useRainbow, bool invertRainbow, Ogre::ColourValue minColor, Ogre::ColourValue maxColor,
                            bool autocomputeIntensity, float minIntensity, float maxIntensity);

  // Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Get grid map layer names.
  std::vector<std::string> getLayerNames();

 private:
  Ogre::SceneNode* frameNode_;
  Ogre::SceneManager* sceneManager_;

  // ManualObject for mesh display.
  Ogre::ManualObject* manualObject_;
  Ogre::MaterialPtr material_;
  std::string materialName_;

  // Lines for mesh.
  boost::shared_ptr<rviz::BillboardLine> meshLines_;

  // Grid map.
  grid_map::GridMap map_;
  bool haveMap_;

  // Helper methods.
  void normalizeIntensity(float& intensity, float minIntensity, float maxIntensity);
  Ogre::ColourValue getRainbowColor(float intensity);
  Ogre::ColourValue getInterpolatedColor(float intensity, Ogre::ColourValue minColor,
                                         Ogre::ColourValue maxColor);
};

}  // namespace
