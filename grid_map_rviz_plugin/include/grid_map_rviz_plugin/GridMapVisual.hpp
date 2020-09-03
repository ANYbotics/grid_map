/*
 * GridMapVisual.cpp
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi, Péter Fankhauser
 *  Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_RVIZ_PLUGIN__GRIDMAPVISUAL_HPP_
#define GRID_MAP_RVIZ_PLUGIN__GRIDMAPVISUAL_HPP_

#ifndef Q_MOC_RUN

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#endif  // Q_MOC_RUN

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace rviz_rendering
{
class BillboardLine;
}

namespace grid_map_rviz_plugin
{

// Visualizes a single grid_map_msgs::msg::GridMap message.
class GridMapVisual
{
public:
  GridMapVisual(Ogre::SceneManager * sceneManager, Ogre::SceneNode * parentNode);
  virtual ~GridMapVisual();

  // Copy the grid map data to map_.
  void setMessage(grid_map_msgs::msg::GridMap::ConstSharedPtr msg);
  // Compute the visualization of map_.
  void computeVisualization(
    float alpha, bool showGridLines, bool flatTerrain, std::string heightLayer, bool flatColor,
    bool noColor, Ogre::ColourValue meshColor, bool mapLayerColor, std::string colorLayer,
    bool useRainbow, bool invertRainbow, Ogre::ColourValue minColor, Ogre::ColourValue maxColor,
    bool autocomputeIntensity, float minIntensity, float maxIntensity);

  // Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  // Get grid map layer names.
  std::vector<std::string> getLayerNames();

private:
  Ogre::SceneNode * frameNode_;
  Ogre::SceneManager * sceneManager_;

  // ManualObject for mesh display.
  Ogre::ManualObject * manualObject_;
  Ogre::MaterialPtr material_;
  std::string materialName_;

  // Lines for mesh.
  boost::shared_ptr<rviz_rendering::BillboardLine> meshLines_;

  // Grid map.
  grid_map::GridMap map_;
  bool haveMap_;

  // Helper methods.
  void normalizeIntensity(float & intensity, float minIntensity, float maxIntensity);
  Ogre::ColourValue getRainbowColor(float intensity);
  Ogre::ColourValue getInterpolatedColor(
    float intensity, Ogre::ColourValue minColor,
    Ogre::ColourValue maxColor);
};

}  // namespace grid_map_rviz_plugin
#endif  // GRID_MAP_RVIZ_PLUGIN__GRIDMAPVISUAL_HPP_
