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
}  // namespace Ogre

namespace rviz {
class BillboardLine;
}

namespace grid_map_rviz_plugin {

// Visualizes a single grid_map_msgs::GridMap message.
class GridMapVisual {
 public:
  using MaskArray = Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>;
  using ColorArray = Eigen::Array<Ogre::ColourValue, Eigen::Dynamic, Eigen::Dynamic>;
  using MatrixConstRef = Eigen::Ref<const grid_map::Matrix>;

  GridMapVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
  virtual ~GridMapVisual();

  // Copy the grid map data to map_.
  void setMessage(const grid_map_msgs::GridMap::ConstPtr& msg);
  // Compute the visualization of map_.

  void computeVisualization(float alpha, bool showGridLines, bool flatTerrain, std::string heightLayer, bool flatColor, bool noColor,
                            Ogre::ColourValue meshColor, bool mapLayerColor, std::string colorLayer, std::string colorMap, bool useColorMap,
                            bool invertColorMap, Ogre::ColourValue minColor, Ogre::ColourValue maxColor, bool autocomputeIntensity,
                            float minIntensity, float maxIntensity, float gridLineThickness, int gridCellDecimation);

  // Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Get grid map layer names.
  std::vector<std::string> getLayerNames();

 private:
  enum class ColoringMethod { FLAT, COLOR_LAYER, INTENSITY_LAYER_MANUAL, INTENSITY_LAYER_COLORMAP, INTENSITY_LAYER_INVERTED_COLORMAP };

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

  // Helper methods.
  bool haveMap_;
  /**
   * Initialized the manualObject if not already initialized and clears all previously added data.
   * @param nVertices An estimate on the number of vertices to be added.
   */
  void initializeAndBeginManualObject(size_t nVertices);

  /**
   * Computes a matrix of color values corresponding to the grid cells. Color is computed depending on the coloringMethod.
   * @param heightData Height values of the cells.
   * @param colorData Values of the layer specified for coloring the mesh.
   * @param coloringMethod The strategy to color, see ColoringMethod.
   * @param colorMap colorMap selected (string). See GridMapColorMaps.hpp
   * @param flatColor Used only if coloringMethod is FLAT
   * @param minIntensity Used for the intensity based coloring methods only.
   * @param maxIntensity Used for the intensity based coloring methods only.
   * @param autocomputeIntensity Wheter to override the values in min and max intensity and compute them based on the provided intensity
   * data.
   * @param minColor Used only if coloringMethod is COLOR_LAYER.
   * @param maxColor Used only if coloringMethod is COLOR_LAYER.
   * @return The color for each cell.
   */
  ColorArray computeColorValues(MatrixConstRef heightData, MatrixConstRef colorData,
                                ColoringMethod coloringMethod, std::string colorMap, Ogre::ColourValue flatColor,
                                double minIntensity, double maxIntensity, bool autocomputeIntensity, Ogre::ColourValue minColor,
                                Ogre::ColourValue maxColor);

  /**
   * Initialized the meshLines_ object. Should be called before adding lines. Sets the drawing style and allocates the buffer.
   * @param cols Number of columns that will be drawn.
   * @param rows Number of rows that will be drawn.
   * @param resolution Resolution of the map. Used to compute the line thickness.
   * @param alpha Line opacity.
   * @param lineWidth line thickness for the mesh lines
   */
  void initializeMeshLines(size_t cols, size_t rows, double resolution, double alpha, double lineWidth);

  /**
   * Computes a mask where all the provided basicLayers are finite. Used to do fast lockups during mesh creation.
   * @param basicLayers The values to check, if they are finite.
   * @return A boolean mask. True indicates that all layers are finite, false indicates that at least one layer is NAN.
   */
  MaskArray computeIsValidMask(std::vector<std::string> basicLayers);

  /**
   * Transforms the intensity into [0,1] range where 0 corresponds to the minIntensity and 1 to maxIntensity. The given value is clipped to
   * that range.
   * @param intensity The intensity value to normalize.
   * @param minIntensity Lower bound.
   * @param maxIntensity Upper bound.
   */
  static void normalizeIntensity(float& intensity, float minIntensity, float maxIntensity);

  /**
   * Copied from rviz/src/rviz/default_plugin/point_cloud_transformers.cpp. Transforms an intensity value in [0,1] range to a rainbow
   * coloring.
   * @param intensity Value to color, should be in [0,1] range, otherwise it is clipped.
   * @return The corresponding rainbow color.
   */
  static Ogre::ColourValue getRainbowColor(float intensity);

  /**
   * Returns a linearly interpolated color between min and max Color with the weighting [0,1] specified by intensity.
   * @param intensity The weighting for interpolation, 0 means minColor, 1 means maxColor.
   * @param minColor The lower color for interpolation.
   * @param maxColor The upper color for interpolation.
   * @return
   */
  Ogre::ColourValue getInterpolatedColor(float intensity, Ogre::ColourValue minColor, Ogre::ColourValue maxColor);

  /**
   * Returns a vector of ogre coordinates. Each coordinate is a vertex for a mesh line.
   * @param i Index of the current point in x.
   * @param j Index of the current point in y.
   * @param gridCellDecimation Integer that defines how many cells to skip between mesh lines. E.g. if n=3,
   *        every third mesh line will be displayed.
   * @param isNthRow Flag to indicate the n-th row, where n = gridCellDecimation.
   * @param isNthCol Flag to indicate the n-th column, where n = gridCellDecimation.
   * @param isLastRow Flag to indicate the last row.
   * @param isLastCol Flag to indicate the last column.
   * @param resolution The resolution.
   * @param topLeft The (x,y) position of the top left corner in the map.
   * @param heightOrFlatData The height data for the elevation (z coordinates).
   * @param isValid Mask of indices with valid data (i.e. not nan or inf)
   * @return
   */
  std::vector<Ogre::Vector3> computeMeshLineVertices(int i, int j, int gridCellDecimation, bool isNthRow, bool isNthCol, bool isLastRow,
                                                     bool isLastCol, double resolution, const grid_map::Position& topLeft,
                                                     const Eigen::ArrayXXf& heightOrFlatData, const MaskArray& isValid) const;
};

}  // namespace grid_map_rviz_plugin
