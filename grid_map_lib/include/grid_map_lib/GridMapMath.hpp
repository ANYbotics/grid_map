/*
 * GridMapMath.hpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>

namespace grid_map_lib {

/*!
 * Gets the position of a cell specified by its index in the map frame.
 * @param[out] position the position of the center of the cell in the map frame.
 * @param[in] index of the cell.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] mapPosition the position of the map.
 * @param[in] resolution the resolution of the map.
 * @param[in] bufferSize the size of the buffer (optional).
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if index not within range of buffer.
 */
bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const Eigen::Array2d& mapLength,
                          const Eigen::Vector2d& mapPosition,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Gets the index of the cell which contains a position in the map frame.
 * @param[out] index of the cell.
 * @param[in] position the position in the map frame.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] mapPosition the position of the map.
 * @param[in] resolution the resolution of the map.
 * @param[in] bufferSize the size of the buffer (optional).
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if position outside of map.
 */
bool getIndexFromPosition(Eigen::Array2i& index,
                          const Eigen::Vector2d& position,
                          const Eigen::Array2d& mapLength,
                          const Eigen::Vector2d& mapPosition,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Checks if position is within the map boundaries.
 * @param[in] position the position which is to be checked.
 * @param[in] mapLength the length of the map.
 * @param[in] mapPosition the position of the map.
 * @return true if position is within map, false otherwise.
 */
bool checkIfPositionWithinMap(const Eigen::Vector2d& position,
                              const Eigen::Array2d& mapLength,
                              const Eigen::Vector2d& mapPosition);

/*!
 * Gets the position of the data structure origin.
 * @param[in] position the position of the map.
 * @param[in] mapLength the map length.
 * @param[out] positionOfOrigin the position of the data structure origin.
 */
void getPositionOfDataStructureOrigin(const Eigen::Vector2d& position,
                                      const Eigen::Array2d& mapLength,
                                      Eigen::Vector2d& positionOfOrigin);

/*!
 * Computes how many cells/indeces the map is moved based on a position shift in
 * the grid map frame. Use this function if you are moving the grid map
 * and want to ensure that the cells match before and after.
 * @param[out] indexShift the corresponding shift of the indices.
 * @param[in] positionShift the desired position shift.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
bool getIndexShiftFromPositionShift(Eigen::Array2i& indexShift,
                                    const Eigen::Vector2d& positionShift,
                                    const double& resolution);

/*!
 * Computes the corresponding position shift from a index shift. Use this function
 * if you are moving the grid map and want to ensure that the cells match
 * before and after.
 * @param[out] positionShift the corresponding shift in position in the grid map frame.
 * @param[in] indexShift the desired shift of the indeces.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
bool getPositionShiftFromIndexShift(Eigen::Vector2d& positionShift,
                                    const Eigen::Array2i& indexShift,
                                    const double& resolution);

/*!
 * Checks if index is within range of the buffer.
 * @param[in] index to check.
 * @param[in] bufferSize the size of the buffer.
 * @return true if index is within, and false if index is outside of the buffer.
 */
bool checkIfIndexWithinRange(const Eigen::Array2i& index, const Eigen::Array2i& bufferSize);

/*!
 * Maps an index that runs out of the range of the circular buffer back into allowed the region.
 * This is the 2d version of mapIndexWithinRange(int&, const int&).
 * @param[in/out] index the indeces that will be mapped into the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void mapIndexWithinRange(Eigen::Array2i& index,
                         const Eigen::Array2i& bufferSize);

/*!
 * Maps an index that runs out of the range of the circular buffer back into allowed the region.
 * @param[in/out] index the index that will be mapped into the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void mapIndexWithinRange(int& index, const int& bufferSize);

/*!
 * Limits (cuts off) the position to lie inside the map.
 * @param[in/out] position the position to be limited.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] mapPosition the position of the map.
 */
void limitPositionToRange(Eigen::Vector2d& position,
                          const Eigen::Array2d& mapLength,
                          const Eigen::Vector2d& mapPosition);

/*!
 * Provides the alignment transformation from the buffer order (outer/inner storage)
 * and the map frame (x/y-coordinate).
 * @return the alignment transformation.
 */
const Eigen::Matrix2i getBufferOrderToMapFrameAlignment();

/*!
 * Given a map and a desired submap (defined by position and size), this function computes
 * various information about the submap. The returned submap might be smaller than the requested
 * size as it respects the boundaries of the map.
 * @param[out] submapTopLeftIndex the top left index of the returned submap.
 * @param[out] submapBufferSize the buffer size of the returned submap.
 * @param[out] submapPosition the position of the submap (center) in the map frame.
 * @param[out] submapLength the length of the submap.
 * @param[out] requestedIndexInSubmap the index in the submap that corresponds to the requested
 *             position of the submap.
 * @param[in] requestedSubmapPosition the requested submap position (center) in the map frame.
 * @param[in] requestedSubmapLength the requested submap length.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] mapPosition the position of the map.
 * @param[in] resolution the resolution of the map.
 * @param[in] bufferSize the buffer size of the map.
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful.
 */
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
                          const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Computes the regions in the circular buffer that make up the data for
 * a requested submap.
 * @param[out] submapIndeces the list of indeces (top-left) for the buffer regions.
 * @param[out] submapSizes the sizes of the buffer regions.
 * @param[in] submapIndex the index (top-left) for the requested submap.
 * @param[in] submapBufferSize the size of the requested submap.
 * @param[in] bufferSize the buffer size of the map.
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if requested submap is not fully contained in the map.
 */
bool getBufferRegionsForSubmap(std::vector<Eigen::Array2i>& submapIndeces,
                               std::vector<Eigen::Array2i>& submapSizes,
                               const Eigen::Array2i& submapIndex,
                               const Eigen::Array2i& submapBufferSize,
                               const Eigen::Array2i& bufferSize,
                               const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Increases the index by one to iterate through the map.
 * Increments either to the neighboring index to the right or to
 * the start of the lower row. Returns false if end of iterations are reached.
 * @param[in/out] index the index in the map that is incremented (corrected for the circular buffer).
 * @param[in] bufferSize the map buffer size.
 * @param[in] bufferStartIndex the map buffer start index.
 * @return true if successfully incremented indeces, false if end of iteration limits are reached.
 */
bool incrementIndex(Eigen::Array2i& index, const Eigen::Array2i& bufferSize,
                    const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Increases the index by one to iterate through the cells of a submap.
 * Increments either to the neighboring index to the right or to
 * the start of the lower row. Returns false if end of iterations are reached.
 *
 * Note: This function does not check if submap actually fits to the map. This needs
 * to be checked before separately.
 *
 * @param[in/out] submapIndex the index in the submap that is incremented.
 * @param[out] index the index in the map that is incremented (corrected for the circular buffer).
 * @param[in] submapTopLefIndex the top left index of the submap.
 * @param[in] submapBufferSize the submap buffer size.
 * @param[in] bufferSize the map buffer size.
 * @param[in] bufferStartIndex the map buffer start index.
 * @return true if successfully incremented indeces, false if end of iteration limits are reached.
 */
bool incrementIndexForSubmap(Eigen::Array2i& submapIndex, Eigen::Array2i& index, const Eigen::Array2i& submapTopLeftIndex,
                             const Eigen::Array2i& submapBufferSize, const Eigen::Array2i& bufferSize,
                             const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Returns the 1d index corresponding to the 2d index for either row- or column-major format.
 * Note: Eigen is defaulting to column-major format.
 * @param[in] index the 2d index.
 * @param[in] bufferSize the map buffer size.
 * @param[in] (optional) rowMajor if the 1d index is generated for row-major format.
 * @return the 1d index.
 */
unsigned int get1dIndexFrom2dIndex(const Eigen::Array2i& index, const Eigen::Array2i& bufferSize, const bool rowMajor);

/*!
 * The definition of the buffer regions.
 */
enum class BufferRegion
{
  TopLeft,
  TopRight,
  BottomLeft,
  BottomRight
};

/*!
 * The definition of the position in the list for the buffer regions.
 */
static std::map<BufferRegion, int> bufferRegionIndeces =
{
{ BufferRegion::TopLeft, 0 },
{ BufferRegion::TopRight, 1 },
{ BufferRegion::BottomLeft, 2 },
{ BufferRegion::BottomRight, 3 } };

} // namespace
