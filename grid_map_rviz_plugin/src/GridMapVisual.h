/*
 * GridMapVisual.cpp
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi
 *  Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef GRID_MAP_VISUAL_H
#define GRID_MAP_VISUAL_H

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSharedPtr.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>

namespace Ogre
{
  class Vector3;
  class Quaternion;
  class ManualObject;
  class ColourValue;
}

namespace rviz
{
  class BillboardLine;
}

namespace grid_map_rviz_plugin
{
  // An instance of GridMapVisual visualizes a single grid_map_msgs::GridMap message
  class GridMapVisual
  {
  public:
    // Constructor
    GridMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

    // Destructor
    virtual ~GridMapVisual();

    // Copy the grid map data to map_
    void setMessage(const grid_map_msgs::GridMap::ConstPtr& msg);
    // Compute the visualization of map_
    void computeVisualization(float alpha,
			      bool show_grid_lines,
			      bool flat_terrain,
			      std::string height_layer,
			      bool flat_color,
			      Ogre::ColourValue mesh_color,
			      std::string color_layer,
			      bool use_rainbow,
			      Ogre::ColourValue min_color,
			      Ogre::ColourValue max_color,
			      bool autocompute_intensity,
			      float min_intensity,
			      float max_intensity);

    // Set the coordinate frame pose
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);
    
    // get grid map layer names
    std::vector<std::string> getLayerNames();

  private:       
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
    
    // ManualObject for mesh display
    Ogre::ManualObject* manual_object_;
    Ogre::MaterialPtr material_;
    std::string material_name_;
    
    // lines for mesh
    boost::shared_ptr<rviz::BillboardLine> mesh_lines_;
    
    // grid map
    grid_map::GridMap map_;
    bool have_map_;
    
    // helper functions
    void normalizeIntensity(float& intensity, float min_intensity, float max_intensity);
    Ogre::ColourValue getRainbowColor(float intensity);
    Ogre::ColourValue getInterpolatedColor(float intensity, 
					   Ogre::ColourValue min_color, 
					   Ogre::ColourValue max_color);
  };

} // end namespace grid_map_rviz_plugin

#endif // GRID_MAP_VISUAL_H
