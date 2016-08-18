/*
 * GridMapVisual.cpp
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi
 *  Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <rviz/uniform_string_stream.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTechnique.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

#include <rviz/ogre_helpers/billboard_line.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_rviz_plugin/GridMapVisual.hpp"

namespace grid_map_rviz_plugin
{

GridMapVisual::GridMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) :
manual_object_(0),
have_map_(false)
{
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  // create BillboardLine object
  mesh_lines_.reset(new rviz::BillboardLine(scene_manager_, frame_node_));
}

GridMapVisual::~GridMapVisual()
{
  // destroy the ManualObject
  scene_manager_->destroyManualObject(manual_object_);
  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName());

  // Destroy the frame node
  scene_manager_->destroySceneNode(frame_node_);
}

void GridMapVisual::setMessage(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  // convert grid map message
  grid_map::GridMapRosConverter::fromMessage(*msg, map_);

  have_map_ = true;
}

void GridMapVisual::computeVisualization(float alpha,
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
					 float max_intensity)
{
  if (!have_map_)
  {
    ROS_DEBUG("Unable to visualize grid map, no map data. Use setMessage() first!");
    return;
  }

  // get list of layers, and check if the requested ones are present
  std::vector<std::string> layer_names = map_.getLayers();
  if (layer_names.size() < 1)
  {
    ROS_DEBUG("Unable to visualize grid map, map must contain at least one layer.");
    return;
  }
  if ((!flat_terrain && std::find(layer_names.begin(), layer_names.end(), height_layer) == layer_names.end()) ||
      (!flat_color && std::find(layer_names.begin(), layer_names.end(), color_layer) == layer_names.end()))
  {
    ROS_DEBUG("Unable to visualize grid map, requested layer(s) not available.");
    return;
  }

  // basic grid map data
  size_t rows = map_.getSize()(0);
  size_t cols = map_.getSize()(1);
  if (rows < 2 || cols < 2)
  {
    ROS_DEBUG("GridMap has not enough cells.");
    return;
  }
  double resolution = map_.getResolution();
  grid_map::Matrix& height_data = map_[flat_terrain ? layer_names[0] : height_layer];
  grid_map::Matrix& color_data = map_[flat_color ? layer_names[0] : color_layer];

  // initialize ManualObject
  if (!manual_object_)
  {
    static uint32_t count = 0;
    rviz::UniformStringStream ss;
    ss << "Mesh" << count++;
    manual_object_ = scene_manager_->createManualObject(ss.str());
    frame_node_->attachObject(manual_object_);

    ss << "Material";
    material_name_ = ss.str();
    material_ = Ogre::MaterialManager::getSingleton().create(material_name_, "rviz");
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->setCullingMode(Ogre::CULL_NONE);
  }

  manual_object_->clear();
  size_t num_vertices = 4 + 6 * (cols * rows - cols - rows);
  manual_object_->estimateVertexCount(num_vertices);
  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  mesh_lines_->clear();
  if (show_grid_lines)
  {
    mesh_lines_->setColor(0.0, 0.0, 0.0, 1.0);
    mesh_lines_->setLineWidth(resolution / 10.0);
    mesh_lines_->setMaxPointsPerLine(2);
    size_t num_lines = rows * (cols - 1) + cols * (rows - 1);
    mesh_lines_->setNumLines(num_lines);
  }
  bool plotted_first_line = false;

  // determine max and min intensity
  // can't use Eigen::minCoeff()/maxCoeff() because map may contain NaN cells
  if (autocompute_intensity && !flat_color)
  {
    min_intensity = 1.0e6;
    max_intensity = -1.0e6;
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
      const grid_map::Index index(*iterator);
      float intensity = color_data(index(0), index(1));
      if (!std::isnan(intensity))
      {
	min_intensity = std::min(min_intensity, intensity);
	max_intensity = std::max(max_intensity, intensity);
      }
    }
  }

  // plot mesh
  for (size_t i = 0; i < rows - 1; ++i)
  {
    for (size_t j = 0; j < cols - 1; ++j)
    {
      bool left_valid = true;
      bool right_valid = true;
      std::vector<Ogre::Vector3> vertices;
      std::vector<Ogre::ColourValue> colors;
      for (size_t k = 0; k < 2; k++)
      {
	for (size_t l = 0; l < 2; l++)
	{
	  grid_map::Position position;
	  grid_map::Index index(i + k, j + l);
	  map_.getPosition(index, position);
	  float height = height_data(index(0), index(1));
	  if (std::isnan(height))
	  {
	    if ((k + l) <= 1)
	      left_valid = false;
	    if ((k + l) >= 1)
	      right_valid = false;
	    vertices.push_back(Ogre::Vector3());
	    colors.push_back(Ogre::ColourValue());
	  }
	  else
	  {
	    vertices.push_back(Ogre::Vector3(position(0), position(1), flat_terrain ? 0.0 : height));
	    if (!flat_color)
	    {
	      float intensity = color_data(index(0), index(1));
	      normalizeIntensity(intensity, min_intensity, max_intensity);
	      Ogre::ColourValue color = use_rainbow ? getRainbowColor(intensity) : getInterpolatedColor(intensity, min_color, max_color);
	      colors.push_back(color);
	    }
	  }
	}
      }
      // upper left triangle
      if (left_valid)
      {
	Ogre::Vector3 normal1 = (vertices[1] - vertices[0]).crossProduct(vertices[2] - vertices[0]);
	normal1.normalise();
	for (size_t m = 0; m < 3; m++)
	{
	  manual_object_->position(vertices[m]);
	  manual_object_->normal(normal1);
	  Ogre::ColourValue color = flat_color ? mesh_color : colors[m];
	  manual_object_->colour(color.r, color.g, color.b, alpha);
	}
      }
      // lower right triangle
      if (right_valid)
      {
	Ogre::Vector3 normal2 = (vertices[2] - vertices[1]).crossProduct(vertices[3] - vertices[1]);
	normal2.normalise();
	for (size_t m = 1; m < 4; m++)
	{
	  manual_object_->position(vertices[m]);
	  manual_object_->normal(normal2);
	  Ogre::ColourValue color = flat_color ? mesh_color : colors[m];
	  manual_object_->colour(color.r, color.g, color.b, alpha);
	}
      }
      // plot grid lines
      if (show_grid_lines)
      {
	if (left_valid)
	{
	  // right line
	  if (plotted_first_line)
	    mesh_lines_->newLine();
	  mesh_lines_->addPoint(vertices[0]);
	  mesh_lines_->addPoint(vertices[1]);
	  plotted_first_line = true;
	  // down line
	  mesh_lines_->newLine();
	  mesh_lines_->addPoint(vertices[0]);
	  mesh_lines_->addPoint(vertices[2]);
	}
	if (i == rows - 2 && right_valid)
	{
	  if (plotted_first_line)
	    mesh_lines_->newLine();
	  mesh_lines_->addPoint(vertices[2]);
	  mesh_lines_->addPoint(vertices[3]);
	  plotted_first_line = true;
	}
	if (j == cols - 2 && right_valid)
	{
	  if (plotted_first_line)
	    mesh_lines_->newLine();
	  mesh_lines_->addPoint(vertices[1]);
	  mesh_lines_->addPoint(vertices[3]);
	  plotted_first_line = true;
	}
      }
    }
  }

  manual_object_->end();
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
  material_->getTechnique(0)->setDepthWriteEnabled(true);
}

void GridMapVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void GridMapVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

std::vector<std::string> GridMapVisual::getLayerNames()
{
  return map_.getLayers();
}

// compute intensity value in the interval [0,1]
void GridMapVisual::normalizeIntensity(float& intensity, float min_intensity, float max_intensity)
{
  intensity = std::min(intensity, max_intensity);
  intensity = std::max(intensity, min_intensity);

  intensity = (intensity - min_intensity) / (max_intensity - min_intensity);
}

// copied from rviz/src/rviz/default_plugin/point_cloud_transformers.cpp
Ogre::ColourValue GridMapVisual::getRainbowColor(float intensity)
{
  intensity = std::min(intensity, 1.0f);
  intensity = std::max(intensity, 0.0f);

  float h = intensity * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if ( !(i&1) ) f = 1 - f; // if i is even
  float n = 1 - f;

  Ogre::ColourValue color;
  if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
  else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
  else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
  else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
  else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;

  return color;
}

// get interpolated color value
Ogre::ColourValue GridMapVisual::getInterpolatedColor(float intensity,
						      Ogre::ColourValue min_color,
						      Ogre::ColourValue max_color)
{
  intensity = std::min(intensity, 1.0f);
  intensity = std::max(intensity, 0.0f);

  Ogre::ColourValue color;
  color.r = intensity * (max_color.r - min_color.r) + min_color.r;
  color.g = intensity * (max_color.g - min_color.g) + min_color.g;
  color.b = intensity * (max_color.b - min_color.b) + min_color.b;

  return color;
}

} // end namespace grid_map_rviz_plugin
