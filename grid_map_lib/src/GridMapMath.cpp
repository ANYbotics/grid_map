/*
 * GridMapMath.cpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/GridMapMath.hpp"

// fabs
#include <cmath>

// Limits
#include <limits>

using namespace Eigen;
using namespace std;

namespace grid_map_lib {

namespace internal {

unsigned int nBufferRegions = 4;

/*!
 * Gets the vector from the center of the map to the origin
 * of the map data structure.
 * @param[out] vectorToOrigin the vector from the center of the map the origin of the map data structure.
 * @param[in] mapLength the lengths in x and y direction.
 * @return true if successful.
 */
inline bool getVectorToOrigin(Eigen::Vector2d& vectorToOrigin, const Eigen::Array2d& mapLength)
{
  vectorToOrigin = (0.5 * mapLength).matrix();
  return true;
}

/*!
 * Gets the vector from the center of the map to the center
 * of the first cell of the map data.
 * @param[out] vectorToFirstCell the vector from the center of the cell to the center of the map.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
inline bool getVectorToFirstCell(Eigen::Vector2d& vectorToFirstCell,
                                 const Eigen::Array2d& mapLength, const double& resolution)
{
  Eigen::Vector2d vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);

  // Vector to center of cell.
  vectorToFirstCell = (vectorToOrigin.array() - 0.5 * resolution).matrix();
  return true;
}

inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation()
{
  return -Matrix2i::Identity();
}

inline Eigen::Matrix2i getMapFrameToBufferOrderTransformation()
{
  return getBufferOrderToMapFrameTransformation().transpose();
}

inline bool checkIfStartIndexAtDefaultPosition(const Eigen::Array2i& bufferStartIndex)
{
  return ((bufferStartIndex == 0).all());
}

inline Eigen::Array2i getBufferIndexFromIndex(
    const Eigen::Array2i& index,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
    return index;

  Array2i bufferIndex = index + bufferStartIndex;
  mapIndexWithinRange(bufferIndex, bufferSize);
  return bufferIndex;
}

inline Eigen::Array2i getIndexFromBufferIndex(
    const Eigen::Array2i& bufferIndex,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
    return bufferIndex;

  Array2i index = bufferIndex - bufferStartIndex;
  mapIndexWithinRange(index, bufferSize);
  return index;
}

inline Eigen::Vector2d getIndexVectorFromIndex(
    const Eigen::Array2i& index,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  Array2i unwrappedIndex;
  unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);
  return (getBufferOrderToMapFrameTransformation() * unwrappedIndex.matrix()).cast<double>();
}

inline Eigen::Array2i getIndexFromIndexVector(
    const Eigen::Vector2d& indexVector,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  Eigen::Array2i index = (getMapFrameToBufferOrderTransformation() * indexVector.cast<int>()).array();
  return getBufferIndexFromIndex(index, bufferSize, bufferStartIndex);
}

inline BufferRegion getMapRegion(const Eigen::Array2i& index, const Eigen::Array2i& bufferStartIndex)
{
  if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::TopLeft;
  if (index[0] >= bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return BufferRegion::TopRight;
  if (index[0] <  bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::BottomLeft;
  if (index[0] <  bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return BufferRegion::BottomRight;
}

} // namespace

using namespace internal;

bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const Eigen::Array2d& mapLength,
                          const Eigen::Vector2d& mapPosition,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex)
{
  if (!checkIfIndexWithinRange(index, bufferSize)) return false;
  Vector2d offset;
  getVectorToFirstCell(offset, mapLength, resolution);
  position = mapPosition + offset + resolution * getIndexVectorFromIndex(index, bufferSize, bufferStartIndex);
  return true;
}

bool getIndexFromPosition(Eigen::Array2i& index,
                          const Eigen::Vector2d& position,
                          const Eigen::Array2d& mapLength,
                          const Eigen::Vector2d& mapPosition,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex)
{
  if (!checkIfPositionWithinMap(position, mapLength, mapPosition)) return false;
  Vector2d offset;
  getVectorToOrigin(offset, mapLength);
  Vector2d indexVector = ((position - offset - mapPosition).array() / resolution).matrix();
  index = getIndexFromIndexVector(indexVector, bufferSize, bufferStartIndex);
  return true;
}

bool checkIfPositionWithinMap(const Eigen::Vector2d& position,
                              const Eigen::Array2d& mapLength,
                              const Eigen::Vector2d& mapPosition)
{
  Vector2d offset;
  getVectorToOrigin(offset, mapLength);
  Vector2d positionTransformed = getMapFrameToBufferOrderTransformation().cast<double>() * (position - mapPosition - offset);

  if (positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0
      && positionTransformed.x() <= mapLength(0) && positionTransformed.y() <= mapLength(1)) {
    return true;
  }
  return false;
}

void getPositionOfDataStructureOrigin(const Eigen::Vector2d& position,
                                      const Eigen::Array2d& mapLength,
                                      Eigen::Vector2d& positionOfOrigin)
{
  Eigen::Vector2d vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);
  positionOfOrigin = position + vectorToOrigin;
}

bool getIndexShiftFromPositionShift(Eigen::Array2i& indexShift,
                                    const Eigen::Vector2d& positionShift,
                                    const double& resolution)
{
  Vector2d indexShiftVectorTemp = (positionShift.array() / resolution).matrix();
  Vector2i indexShiftVector;

  for (int i = 0; i < indexShiftVector.size(); i++) {
    indexShiftVector[i] = static_cast<int>(indexShiftVectorTemp[i] + 0.5 * (indexShiftVectorTemp[i] > 0 ? 1 : -1));
  }

  indexShift = (getMapFrameToBufferOrderTransformation() * indexShiftVector).array();
  return true;
}

bool getPositionShiftFromIndexShift(Eigen::Vector2d& positionShift,
                                    const Eigen::Array2i& indexShift,
                                    const double& resolution)
{
  positionShift = (getBufferOrderToMapFrameTransformation() * indexShift.matrix()).cast<double>() * resolution;
  return true;
}

bool checkIfIndexWithinRange(const Eigen::Array2i& index, const Eigen::Array2i& bufferSize)
{
  if (index[0] >= 0 && index[1] >= 0 && index[0] < bufferSize[0] && index[1] < bufferSize[1])
  {
    return true;
  }
  return false;
}

void mapIndexWithinRange(Eigen::Array2i& index,
                         const Eigen::Array2i& bufferSize)
{
  for (int i = 0; i < index.size(); i++) {
    mapIndexWithinRange(index[i], bufferSize[i]);
  }
}

void mapIndexWithinRange(int& index, const int& bufferSize)
{
  if (index < 0) index += ((-index / bufferSize) + 1) * bufferSize;
  index = index % bufferSize;
}

void limitPositionToRange(Eigen::Vector2d& position, const Eigen::Array2d& mapLength, const Eigen::Vector2d& mapPosition)
{
  Vector2d vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);
  Vector2d positionShifted = position - mapPosition + vectorToOrigin;

  // We have to make sure to stay inside the map.
  for (int i = 0; i < positionShifted.size(); i++) {

    double epsilon = 10.0 * numeric_limits<double>::epsilon(); // TODO Why is the factor 10 necessary.
    if (std::fabs(position(i)) > 1.0) epsilon *= std::fabs(position(i));

    if (positionShifted(i) <= 0) {
      positionShifted(i) = epsilon;
      continue;
    }
    if (positionShifted(i) >= mapLength(i)) {
      positionShifted(i) = mapLength(i) - epsilon;
      continue;
    }
  }

  position = positionShifted + mapPosition - vectorToOrigin;
}

const Eigen::Matrix2i getBufferOrderToMapFrameAlignment()
{
  return getBufferOrderToMapFrameTransformation().array().abs().matrix();
}

bool getSubmapInformation(Eigen::Array2i& submapTopLeftIndex,
                          Eigen::Array2i& submapBufferSize,
                          Eigen::Vector2d& submapPosition,
                          Eigen::Array2d& submapLength,
                          Eigen::Array2i& requestedIndexInSubmap,
                          const Eigen::Vector2d& requestedSubmapPosition,
                          const Eigen::Vector2d& requestedSubmapLength,
                          const Eigen::Array2d& mapLength,
                          const Eigen::Vector2d& mapPosition,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex)
{
  // (Top left / bottom right corresponds to the position in the matrix, not the map frame)
  Matrix2d transform = getMapFrameToBufferOrderTransformation().cast<double>();

  // Corners of submap.
  Vector2d topLeftPosition = requestedSubmapPosition - transform * 0.5 * requestedSubmapLength;
  limitPositionToRange(topLeftPosition, mapLength, mapPosition);
  if(!getIndexFromPosition(submapTopLeftIndex, topLeftPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex)) return false;
  Array2i topLeftIndex;
  topLeftIndex = getIndexFromBufferIndex(submapTopLeftIndex, bufferSize, bufferStartIndex);

  Vector2d bottomRightPosition = requestedSubmapPosition + transform * 0.5 * requestedSubmapLength;
  limitPositionToRange(bottomRightPosition, mapLength, mapPosition);
  Array2i bottomRightIndex;
  if(!getIndexFromPosition(bottomRightIndex, bottomRightPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex)) return false;
  bottomRightIndex = getIndexFromBufferIndex(bottomRightIndex, bufferSize, bufferStartIndex);

  // Get the position of the top left corner of the generated submap.
  Vector2d topLeftCorner;
  if(!getPositionFromIndex(topLeftCorner, submapTopLeftIndex, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex)) return false;
  topLeftCorner -= transform * Vector2d::Constant(0.5 * resolution);

  // Size of submap.
  submapBufferSize = bottomRightIndex - topLeftIndex + Array2i::Ones();

  // Length of the submap.
  submapLength = submapBufferSize.cast<double>() * resolution;

  // Position of submap.
  Vector2d vectorToSubmapOrigin;
  getVectorToOrigin(vectorToSubmapOrigin, submapLength);
  submapPosition = topLeftCorner - vectorToSubmapOrigin;

  // Get the index of the cell which corresponds the requested
  // position of the submap.
  if(!getIndexFromPosition(requestedIndexInSubmap, requestedSubmapPosition, submapLength, submapPosition, resolution, submapBufferSize)) return false;

  return true;
}

bool getBufferRegionsForSubmap(std::vector<Eigen::Array2i>& submapIndeces,
                               std::vector<Eigen::Array2i>& submapSizes,
                               const Eigen::Array2i& submapIndex,
                               const Eigen::Array2i& submapBufferSize,
                               const Eigen::Array2i& bufferSize,
                               const Eigen::Array2i& bufferStartIndex)
{
  if ((getIndexFromBufferIndex(submapIndex, bufferSize, bufferStartIndex) + submapBufferSize > bufferSize).any()) return false;

  submapIndeces.clear();
  submapIndeces.resize(nBufferRegions, Array2i::Zero());
  submapSizes.clear();
  submapSizes.resize(nBufferRegions, Array2i::Zero());

  Array2i bottomRightIndex = submapIndex + submapBufferSize - Array2i::Ones();
  mapIndexWithinRange(bottomRightIndex, bufferSize);

  BufferRegion mapRegionOfTopLeft = getMapRegion(submapIndex, bufferStartIndex);
  BufferRegion mapRegionOfBottomRight = getMapRegion(bottomRightIndex, bufferStartIndex);

  unsigned int topLeft = bufferRegionIndeces[BufferRegion::TopLeft];
  unsigned int topRight = bufferRegionIndeces[BufferRegion::TopRight];
  unsigned int bottomLeft= bufferRegionIndeces[BufferRegion::BottomLeft];
  unsigned int bottomRight= bufferRegionIndeces[BufferRegion::BottomRight];

  if (mapRegionOfTopLeft == BufferRegion::TopLeft) {

    if (mapRegionOfBottomRight == BufferRegion::TopLeft) {
      submapIndeces[topLeft] = submapIndex;
      submapSizes[topLeft] = submapBufferSize;
      return true;
    }

    if (mapRegionOfBottomRight == BufferRegion::TopRight) {
      submapIndeces[topLeft] = submapIndex;
      submapSizes[topLeft](0) = submapBufferSize(0);
      submapSizes[topLeft](1) = bufferSize(1) - submapIndex(1);

      submapIndeces[topRight](0) = submapIndex(0);
      submapIndeces[topRight](1) = 0;
      submapSizes[topRight](0) = submapBufferSize(0);
      submapSizes[topRight](1) = submapBufferSize(1) - submapSizes[topLeft](1);
      return true;
    }

    if (mapRegionOfBottomRight == BufferRegion::BottomLeft) {
      submapIndeces[topLeft] = submapIndex;
      submapSizes[topLeft](0) = bufferSize(0) - submapIndex(0);
      submapSizes[topLeft](1) = submapBufferSize(1);

      submapIndeces[bottomLeft](0) = 0;
      submapIndeces[bottomLeft](1) = submapIndex(1);
      submapSizes[bottomLeft](0) = submapBufferSize(0) - submapSizes[topLeft](0);
      submapSizes[bottomLeft](1) = submapBufferSize(1);
      return true;
    }

    if (mapRegionOfBottomRight == BufferRegion::BottomRight) {
      submapIndeces[topLeft] = submapIndex;
      submapSizes[topLeft](0) = bufferSize(0) - submapIndex(0);
      submapSizes[topLeft](1) = bufferSize(1) - submapIndex(1);

      submapIndeces[topRight](0) = submapIndex(0);
      submapIndeces[topRight](1) = 0;
      submapSizes[topRight](0) = bufferSize(0) - submapIndex(0);
      submapSizes[topRight](1) = submapBufferSize(1) - submapSizes[topLeft](1);

      submapIndeces[bottomLeft](0) = 0;
      submapIndeces[bottomLeft](1) = submapIndex(1);
      submapSizes[bottomLeft](0) = submapBufferSize(0) - submapSizes[topLeft](0);
      submapSizes[bottomLeft](1) = bufferSize(1) - submapIndex(1);

      submapIndeces[bottomRight] = Array2i::Zero();
      submapSizes[bottomRight](0) = submapSizes[bottomLeft](0);
      submapSizes[bottomRight](1) = submapSizes[topRight](1);
      return true;
    }

  } else if (mapRegionOfTopLeft == BufferRegion::TopRight) {

    if (mapRegionOfBottomRight == BufferRegion::TopRight) {
      submapIndeces[topRight] = submapIndex;
      submapSizes[topRight] = submapBufferSize;
      return true;
    }

    if (mapRegionOfBottomRight == BufferRegion::BottomRight) {
      submapIndeces[topRight] = submapIndex;
      submapSizes[topRight](0) = bufferSize(0) - submapIndex(0);
      submapSizes[topRight](1) = submapBufferSize(1);

      submapIndeces[bottomRight](0) = 0;
      submapIndeces[bottomRight](1) = submapIndex(1);
      submapSizes[bottomRight](0) = submapBufferSize(0) - submapSizes[topRight](0);
      submapSizes[bottomRight](1) = submapBufferSize(1);
      return true;
    }

  } else if (mapRegionOfTopLeft == BufferRegion::BottomLeft) {

    if (mapRegionOfBottomRight == BufferRegion::BottomLeft) {
      submapIndeces[bottomLeft] = submapIndex;
      submapSizes[bottomLeft] = submapBufferSize;
      return true;
    }

    if (mapRegionOfBottomRight == BufferRegion::BottomRight) {
      submapIndeces[bottomLeft] = submapIndex;
      submapSizes[bottomLeft](0) = submapBufferSize(0);
      submapSizes[bottomLeft](1) = bufferSize(1) - submapIndex(1);

      submapIndeces[bottomRight](0) = submapIndex(0);
      submapIndeces[bottomRight](1) = 0;
      submapSizes[bottomRight](0) = submapBufferSize(0);
      submapSizes[bottomRight](1) = submapBufferSize(1) - submapSizes[bottomLeft](1);
      return true;
    }

  } else if (mapRegionOfTopLeft == BufferRegion::BottomRight) {

    if (mapRegionOfBottomRight == BufferRegion::BottomRight) {
      submapIndeces[bottomRight] = submapIndex;
      submapSizes[bottomRight] = submapBufferSize;
      return true;
    }

  }

  return false;
}

bool incrementIndex(Eigen::Array2i& index, const Eigen::Array2i& bufferSize,
                    const Eigen::Array2i& bufferStartIndex)
{
  Eigen::Array2i unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);

  // Increment index.
  if (unwrappedIndex(1) + 1 < bufferSize(1)) {
    // Same row.
    unwrappedIndex[1]++;
  } else {
    // Next row.
    unwrappedIndex[0]++;
    unwrappedIndex[1] = 0;
  }

  // End of iterations reached.
  if (!checkIfIndexWithinRange(unwrappedIndex, bufferSize)) return false;

  // Return true iterated index.
  index = getBufferIndexFromIndex(unwrappedIndex, bufferSize, bufferStartIndex);
  return true;
}

bool incrementIndexForSubmap(Eigen::Array2i& submapIndex, Eigen::Array2i& index, const Eigen::Array2i& submapTopLeftIndex,
                             const Eigen::Array2i& submapBufferSize, const Eigen::Array2i& bufferSize,
                             const Eigen::Array2i& bufferStartIndex)
{
  // Copy the data first, only copy it back if everything is within range.
  Array2i tempIndex = index;
  Array2i tempSubmapIndex = submapIndex;

  // Increment submap index.
  if (tempSubmapIndex[1] + 1 < submapBufferSize[1]) {
    // Same row.
    tempSubmapIndex[1]++;
  } else {
    // Next row.
    tempSubmapIndex[0]++;
    tempSubmapIndex[1] = 0;
  }

  // End of iterations reached.
  if (!checkIfIndexWithinRange(tempSubmapIndex, submapBufferSize)) return false;

  // Get corresponding index in map.
  Array2i unwrappedSubmapTopLeftIndex = getIndexFromBufferIndex(submapTopLeftIndex, bufferSize, bufferStartIndex);
  tempIndex = getBufferIndexFromIndex(unwrappedSubmapTopLeftIndex + tempSubmapIndex, bufferSize, bufferStartIndex);

  // Copy data back.
  index = tempIndex;
  submapIndex = tempSubmapIndex;
  return true;
}

unsigned int get1dIndexFrom2dIndex(const Eigen::Array2i& index, const Eigen::Array2i& bufferSize, const bool rowMajor)
{
  if (!rowMajor) return index(1) * bufferSize(0) + index(0);
  return index(0) * bufferSize(1) + index(1);
}

}  // namespace
