/*
 * GridMapVisual.cpp
 *
 *  Created on: Aug 3, 2016
 *  Author: Philipp Krüsi, Péter Fankhauser
 *  Institute: ETH Zurich, ANYbotics
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
#include <grid_map_core/GridMapMath.hpp>
#include "grid_map_rviz_plugin/GridMapVisual.hpp"

namespace grid_map_rviz_plugin {

GridMapVisual::GridMapVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode)
    : manualObject_(0),
      haveMap_(false)
{
  sceneManager_ = sceneManager;
  frameNode_ = parentNode->createChildSceneNode();

  // Create BillboardLine object.
  meshLines_.reset(new rviz::BillboardLine(sceneManager_, frameNode_));
}

GridMapVisual::~GridMapVisual()
{
  // Destroy the ManualObject.
  sceneManager_->destroyManualObject(manualObject_);
  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName());

  // Destroy the frame node.
  sceneManager_->destroySceneNode(frameNode_);
}

void GridMapVisual::setMessage(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  // Convert grid map message.
  grid_map::GridMapRosConverter::fromMessage(*msg, map_);
  haveMap_ = true;
}

void GridMapVisual::computeVisualization(float alpha, bool showGridLines, bool flatTerrain, std::string heightLayer,
                                         bool flatColor, bool noColor, Ogre::ColourValue meshColor, bool mapLayerColor,
                                         std::string colorLayer, bool useRainbow, bool invertRainbow,
                                         Ogre::ColourValue minColor, Ogre::ColourValue maxColor,
                                         bool autocomputeIntensity, float minIntensity, float maxIntensity)
{
  if (!haveMap_) {
    ROS_DEBUG("Unable to visualize grid map, no map data. Use setMessage() first!");
    return;
  }

  // Get list of layers and check if the requested ones are present.
  std::vector<std::string> layerNames = map_.getLayers();
  if (layerNames.size() < 1) {
    ROS_DEBUG("Unable to visualize grid map, map must contain at least one layer.");
    return;
  }
  if ((!flatTerrain && !map_.exists(heightLayer)) || (!noColor && !flatColor && !map_.exists(colorLayer))) {
    ROS_DEBUG("Unable to visualize grid map, requested layer(s) not available.");
    return;
  }

  // Convert to simple format, makes things easier.
  map_.convertToDefaultStartIndex();

  // Basic grid map data.
  const size_t rows = map_.getSize()(0);
  const size_t cols = map_.getSize()(1);
  if (rows < 2 || cols < 2) {
    ROS_DEBUG("GridMap has not enough cells.");
    return;
  }
  const double resolution = map_.getResolution();
  const grid_map::Matrix& heightData = map_[flatTerrain ? layerNames[0] : heightLayer];
  const grid_map::Matrix& colorData = map_[flatColor ? layerNames[0] : colorLayer];

  // initialize ManualObject
  if (!manualObject_) {
    static uint32_t count = 0;
    rviz::UniformStringStream ss;
    ss << "Mesh" << count++;
    manualObject_ = sceneManager_->createManualObject(ss.str());
    frameNode_->attachObject(manualObject_);

    ss << "Material";
    materialName_ = ss.str();
    material_ = Ogre::MaterialManager::getSingleton().create(materialName_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->setCullingMode(Ogre::CULL_NONE);
  }

  manualObject_->clear();
  size_t nVertices = 4 + 6 * (cols * rows - cols - rows);
  manualObject_->estimateVertexCount(nVertices);
  manualObject_->begin(materialName_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  meshLines_->clear();
  if (showGridLines) {
    meshLines_->setColor(0.0, 0.0, 0.0, alpha);
    meshLines_->setLineWidth(resolution / 10.0);
    meshLines_->setMaxPointsPerLine(2);
    // In the algorithm below, we have to account for max. 4 lines per cell.
    size_t nLines = 2 * (rows * (cols - 1) + cols * (rows - 1));
    meshLines_->setNumLines(nLines);
  }

  // Determine max and min intensity.
  if (autocomputeIntensity && !flatColor && !mapLayerColor) {
    minIntensity = colorData.minCoeffOfFinites();
    maxIntensity = colorData.maxCoeffOfFinites();
  }

  if (!map_.hasBasicLayers()) map_.setBasicLayers({heightLayer});

  // Plot mesh.
  for (size_t i = 0; i < rows - 1; ++i) {
    for (size_t j = 0; j < cols - 1; ++j) {
      std::vector<Ogre::Vector3> vertices;
      std::vector<Ogre::ColourValue> colors;
      for (size_t k = 0; k < 2; k++) {
        for (size_t l = 0; l < 2; l++) {
          grid_map::Position position;
          grid_map::Index index(i + k, j + l);
          if (!map_.isValid(index)) continue;

          map_.getPosition(index, position);
          float height = heightData(index(0), index(1));
          vertices.push_back(Ogre::Vector3(position(0), position(1), flatTerrain ? 0.0 : height));
          if (!flatColor) {
            float color = colorData(index(0), index(1));
            Ogre::ColourValue colorValue;
            if (mapLayerColor) {
              Eigen::Vector3f colorVectorRGB;
              grid_map::colorValueToVector(color, colorVectorRGB);
              colorValue = Ogre::ColourValue(colorVectorRGB(0), colorVectorRGB(1), colorVectorRGB(2));
            } else {
              normalizeIntensity(color, minIntensity, maxIntensity);
              colorValue = useRainbow ? (invertRainbow ? getRainbowColor(1.0f - color)
                                                       : getRainbowColor(color))
                                      : getInterpolatedColor(color, minColor, maxColor);
            }
            colors.push_back(colorValue);
          }
        }
      }

      // Plot triangles if we have enough vertices.
      if (vertices.size() > 2) {
        Ogre::Vector3 normal = vertices.size() == 4
                               ? (vertices[3] - vertices[0]).crossProduct(vertices[2] -vertices[1])
                               : (vertices[2] - vertices[1]).crossProduct(vertices[1] - vertices[0]);
        normal.normalise();
        // Create one or two triangles from the vertices depending on how many vertices we have.
        if (!noColor) {
          for (size_t m = 1; m < vertices.size() - 1; m++) {
            manualObject_->position(vertices[m-1]);
            manualObject_->normal(normal);
            Ogre::ColourValue color = flatColor ? meshColor : colors[m-1];
            manualObject_->colour(color.r, color.g, color.b, alpha);

            manualObject_->position(vertices[m]);
            manualObject_->normal(normal);
            color = flatColor ? meshColor : colors[m];
            manualObject_->colour(color.r, color.g, color.b, alpha);

            manualObject_->position(vertices[m+1]);
            manualObject_->normal(normal);
            color = flatColor ? meshColor : colors[m+1];
            manualObject_->colour(color.r, color.g, color.b, alpha);
          }
        }

        // plot grid lines
        if (showGridLines) {
          meshLines_->addPoint(vertices[0]);
          meshLines_->addPoint(vertices[1]);
          meshLines_->newLine();

          if (vertices.size() == 3) {
            meshLines_->addPoint(vertices[1]);
            meshLines_->addPoint(vertices[2]);
            meshLines_->newLine();
          } else {
            meshLines_->addPoint(vertices[1]);
            meshLines_->addPoint(vertices[3]);
            meshLines_->newLine();

            meshLines_->addPoint(vertices[3]);
            meshLines_->addPoint(vertices[2]);
            meshLines_->newLine();
          }

          meshLines_->addPoint(vertices[2]);
          meshLines_->addPoint(vertices[0]);
          meshLines_->newLine();
        }
      }
    }
  }

  manualObject_->end();
  material_->getTechnique(0)->setLightingEnabled(false);

  if (alpha < 0.9998) {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(false);
  } else {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
  }
}

void GridMapVisual::setFramePosition(const Ogre::Vector3& position)
{
  frameNode_->setPosition(position);
}

void GridMapVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frameNode_->setOrientation(orientation);
}

std::vector<std::string> GridMapVisual::getLayerNames()
{
  return map_.getLayers();
}

// Compute intensity value in the interval [0,1].
void GridMapVisual::normalizeIntensity(float& intensity, float min_intensity, float max_intensity)
{
  intensity = std::min(intensity, max_intensity);
  intensity = std::max(intensity, min_intensity);
  intensity = (intensity - min_intensity) / (max_intensity - min_intensity);
}

// Copied from rviz/src/rviz/default_plugin/point_cloud_transformers.cpp.
Ogre::ColourValue GridMapVisual::getRainbowColor(float intensity)
{
  intensity = std::min(intensity, 1.0f);
  intensity = std::max(intensity, 0.0f);

  float h = intensity * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  float n = 1 - f;

  Ogre::ColourValue color;
  if (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
  else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
  else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
  else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
  else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;

  return color;
}

// Get interpolated color value.
Ogre::ColourValue GridMapVisual::getInterpolatedColor(float intensity, Ogre::ColourValue min_color,
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

}  // namespace
