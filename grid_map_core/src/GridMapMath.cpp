/*
 * GridMapMath.cpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMapMath.hpp"

// fabs
#include <cmath>

// Limits
#include <limits>

using namespace Eigen;
using namespace std;

namespace grid_map {

namespace internal {

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

inline BufferRegion::Quadrant getQuadrant(const Eigen::Array2i& index, const Eigen::Array2i& bufferStartIndex)
{
  if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::Quadrant::TopLeft;
  if (index[0] >= bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return BufferRegion::Quadrant::TopRight;
  if (index[0] <  bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::Quadrant::BottomLeft;
  if (index[0] <  bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return BufferRegion::Quadrant::BottomRight;
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
      && positionTransformed.x() < mapLength(0) && positionTransformed.y() < mapLength(1)) {
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

bool getBufferRegionsForSubmap(std::vector<BufferRegion>& submapBufferRegions,
                               const Index& submapIndex,
                               const Size& submapBufferSize,
                               const Size& bufferSize,
                               const Index& bufferStartIndex)
{
  if ((getIndexFromBufferIndex(submapIndex, bufferSize, bufferStartIndex) + submapBufferSize > bufferSize).any()) return false;

  submapBufferRegions.clear();

  Index bottomRightIndex = submapIndex + submapBufferSize - Index::Ones();
  mapIndexWithinRange(bottomRightIndex, bufferSize);

  BufferRegion::Quadrant quadrantOfTopLeft = getQuadrant(submapIndex, bufferStartIndex);
  BufferRegion::Quadrant quadrantOfBottomRight = getQuadrant(bottomRightIndex, bufferStartIndex);

  if (quadrantOfTopLeft == BufferRegion::Quadrant::TopLeft) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopLeft) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::TopLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopRight) {
      Size topLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index topRightIndex(submapIndex(0), 0);
      Size topRightSize(submapBufferSize(0), submapBufferSize(1) - topLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(topRightIndex, topRightSize, BufferRegion::Quadrant::TopRight));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomLeft) {
      Size topLeftSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index bottomLeftIndex(0, submapIndex(1));
      Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomLeftIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {
      Size topLeftSize(bufferSize(0) - submapIndex(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index topRightIndex(submapIndex(0), 0);
      Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1) - topLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(topRightIndex, topRightSize, BufferRegion::Quadrant::TopRight));

      Index bottomLeftIndex(0, submapIndex(1));
      Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(bottomLeftIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));

      Index bottomRightIndex = Array2i::Zero();
      Size bottomRightSize(bottomLeftSize(0), topRightSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }

  } else if (quadrantOfTopLeft == BufferRegion::Quadrant::TopRight) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopRight) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::TopRight));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {

      Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topRightSize, BufferRegion::Quadrant::TopRight));

      Index bottomRightIndex(0, submapIndex(1));
      Size bottomRightSize(submapBufferSize(0) - topRightSize(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }

  } else if (quadrantOfTopLeft == BufferRegion::Quadrant::BottomLeft) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomLeft) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::BottomLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {
      Size bottomLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));

      Index bottomRightIndex(submapIndex(0), 0);
      Size bottomRightSize(submapBufferSize(0), submapBufferSize(1) - bottomLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }

  } else if (quadrantOfTopLeft == BufferRegion::Quadrant::BottomRight) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::BottomRight));
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

Index getIndexFromBufferIndex(const Index& bufferIndex, const Size& bufferSize,
                              const Index& bufferStartIndex)
{
  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
    return bufferIndex;

  Array2i index = bufferIndex - bufferStartIndex;
  mapIndexWithinRange(index, bufferSize);
  return index;
}

size_t getLinearIndexFromIndex(const Index& index, const Size& bufferSize, const bool rowMajor)
{
  if (!rowMajor) return index(1) * bufferSize(0) + index(0);
  return index(0) * bufferSize(1) + index(1);
}

Index getIndexFromLinearIndex(const size_t linearIndex, const Size& bufferSize, const bool rowMajor)
{
  if (!rowMajor) return Index((int)linearIndex % bufferSize(0), (int)linearIndex / bufferSize(0));
  return Index((int)linearIndex / bufferSize(1), (int)linearIndex % bufferSize(1));
}

void getIndicesForRegion(const Index& regionIndex, const Size& regionSize,
                         std::vector<Index> indices)
{
//  for (int i = line.index_; col < line.endIndex(); col++) {
//    for (int i = 0; i < getSize()(0); i++) {
//
//    }
//  }
}

void getIndicesForRegions(const std::vector<Index>& regionIndeces, const Size& regionSizes,
                          std::vector<Index> indices)
{
}

}  // namespace

