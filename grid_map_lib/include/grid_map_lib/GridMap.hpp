/*
 * GridMap.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// STL
#include <vector>
#include <unordered_map>

// Eigen
#include <Eigen/Core>

namespace grid_map_lib {

/*!
 * Grid map managing multiple overlaying maps holding float values.
 * Data structure implemented as two-dimensional circular buffer so map
 * can be moved efficiently.
 *
 * Data is defined with string keys. Examples are:
 * - "elevation"
 * - "variance"
 * - "color"
 * - "quality"
 * - "surface_normal_x", "surface_normal_y", "surface_normal_z"
 * etc.
 */
class GridMap
{
 public:
  /*!
   * Constructor.
   * @param types a vector of strings containing the definition/description of the data.
   */
  GridMap(const std::vector<std::string>& types);

  /*!
   * Destructor.
   */
  virtual ~GridMap();

  /*!
   * Set the geometry of the grid map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the grid map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the grid map in the grid map frame [m].
   */
  void setGeometry(const Eigen::Array2d& length, const double resolution, const Eigen::Vector2d& position);

  /*!
   * Set the types that are required to be set NAN when clearing cells of the map.
   * By default the list contains all types.
   * @param clearTypes the list of types that are required to be set to NAN when clearing cells of the map.
   */
  void setClearTypes(const std::vector<std::string>& clearTypes);

  /*!
   * Add a new data.
   * @param type the type identifier of the data.
   * @param data the data to be added.
   */
  void add(const std::string& type, const Eigen::MatrixXf& data);

  /*!
   * Checks if data type exists.
   * @param type the type identifier of the data.
   * @return true if type exists, false otherwise.
   */
  bool exists(const std::string& type) const;

  /*!
   * Returns the grid map data for a type.
   * @param type the data to be returned.
   * @return grid map data.
   */
  const Eigen::MatrixXf& get(const std::string& type) const;

  /*!
   * Returns the grid map data for a type as non-const. Use this function
   * with great care!
   * @param type the data to be returned.
   * @return grid map data.
   */
  Eigen::MatrixXf& get(const std::string& type);

  /*!
   * Removes the data for a certain kind.
   * @param type the data to be removed.
   * @return true if successful.
   */
  bool remove(const std::string& type);

  /*!
   * Get cell data at requested position.
   * @param type the type of the map to be accessed.
   * @param position the requested position.
   * @return the data of the cell.
   */
  float& atPosition(const std::string& type, const Eigen::Vector2d& position);

  /*!
   * Get cell data at requested position. Const version form above.
   * @param type the type of the map to be accessed.
   * @param position the requested position.
   * @return the data of the cell.
   */
  float atPosition(const std::string& type, const Eigen::Vector2d& position) const;

  /*!
   * Get cell data for requested index.
   * @param type the type of the map to be accessed.
   * @param index the requested index.
   * @return the data of the cell.
   */
  float& at(const std::string& type, const Eigen::Array2i& index);

  /*!
   * Get cell data for requested index. Const version form above.
   * @param type the type of the map to be accessed.
   * @param index the requested index.
   * @return the data of the cell.
   */
  float at(const std::string& type, const Eigen::Array2i& index) const;

  /*!
   * Gets the corresponding cell index for a position.
   * @param[in] position the requested position.
   * @param[out] index the corresponding index.
   * @return true if successful, false if position outside of map.
   */
  bool getIndex(const Eigen::Vector2d& position, Eigen::Array2i& index) const;

  /*!
   * Gets the 2d position of cell specified by the index (x, y of cell position) in
   * the grid map frame.
   * @param[in] index the index of the requested cell.
   * @param[out] position the position of the data point in the parent frame.
   * @return true if successful, false if index not within range of buffer.
   */
  bool getPosition(const Eigen::Array2i& index, Eigen::Vector2d& position) const;

  /*!
   * Check if position is within the map boundaries.
   * @param position the position to be checked.
   * @return true if position is within map, false otherwise.
   */
  bool isInside(const Eigen::Vector2d& position);

  /*!
   * Checks if cell at index is a valid, i.e. if all clearTypes are finite.
   * @param index the index to check.
   * @return true if cell is valid, false otherwise.
   */
  bool isValid(const Eigen::Array2i& index) const;

  /*!
   * Checks if cell at index is a valid (finite) for certain types.
   * @param index the index to check.
   * @param types the types to be checked for validity.
   * @return true if cell is valid, false otherwise.
   */
  bool isValid(const Eigen::Array2i& index, const std::vector<std::string>& types) const;

  /*!
   * Gets the 3d position of a data point (x, y of cell position & cell value as z) in
   * the grid map frame. This makes sense for data types such as elevation.
   * @param type the type of the map to be accessed.
   * @param index the index of the requested cell.
   * @param position the position of the data point in the parent frame.
   * @return true if successful, false if no valid data available.
   */
  bool getPosition3d(const std::string& type, const Eigen::Array2i& index, Eigen::Vector3d& position) const;

  /*!
   * Gets the 3d vector of three data types with suffixes 'x', 'y', and 'z'.
   * @param typePrefix the prefix for the type to bet get as vector.
   * @param index the index of the requested cell.
   * @param vector the vector with the values of the data type.
   * @return true if successful, false if no valid data available.
   */
  bool getVector(const std::string& typePrefix, const Eigen::Array2i& index, Eigen::Vector3d& vector) const;

  /*!
   * Gets a submap from the map. The requested submap is specified with the requested
   * location and length.
   * @param[in] position the requested position of the submap (usually the center).
   * @param[in] length the requested length of the submap.
   * @param[out] indexInSubmap the index of the requested position in the submap.
   * @param[out] isSuccess true if successful, false otherwise.
   * @return submap (is empty if success is false).
   */
  GridMap getSubmap(const Eigen::Vector2d& position, const Eigen::Array2d& length, Eigen::Array2i& indexInSubmap, bool& isSuccess);

  /*!
   * Move the grid map w.r.t. to the grid map frame. Use this to move the grid map
   * boundaries without moving the grid map data. Takes care of all the data handling,
   * such that the grid map data is stationary in the grid map frame.
   * @param position the new location of the grid map in the map frame.
   */
  void move(const Eigen::Vector2d& position);

  /*!
   * Clears all cells (set to NAN) for all types of clearTypes.
   * Header information (geometry etc.) remains valid.
   */
  void clear();

  /*!
   * Clears all cells of the grid map (less efficient than clear()).
   * Header information (geometry etc.) remains valid.
   */
  void clearAll();

  /*!
   * Set the timestamp of the grid map.
   * @param timestamp the timestamp to set (in  nanoseconds).
   */
  void setTimestamp(const uint64_t& timestamp);

  /*!
   * Get the timestamp of the grid map.
   * @return timestamp in nanoseconds.
   */
  const uint64_t& getTimestamp() const;

  /*!
   * Resets the timestamp of the grid map (to zero).
   */
  void resetTimestamp();

  /*!
   * Set the frame id of the grid map.
   * @param frameId the frame id to set.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frameId of the grid map.
   * @return frameId.
   */
  const std::string& getFrameId() const;

  /*!
   * Get the side length of the grid map.
   * @return side length of the grid map.
   */
  const Eigen::Array2d& getLength() const;

  /*!
   * Get the 2d position of the grid map in the grid map frame.
   * @return position of the grid map in the grid map frame.
   */
  const Eigen::Vector2d& getPosition() const;

  /*!
   * Get the resolution of the grid map.
   * @return resolution of the grid map in the xy plane [m/cell].
   */
  double getResolution() const;

  /*!
   * Get the buffer size (rows and cols of the data structure).
   * @return buffer size.
   */
  const Eigen::Array2i& getBufferSize() const;

  /*!
   * Get the start index of the circular buffer.
   * @return buffer start index.
   */
  const Eigen::Array2i& getBufferStartIndex() const;

 protected:

  /*!
   * Emtpy constructor.
   */
  GridMap();

  /*!
   * Clear a number of columns of the grid map.
   * @param index the left index for the columns to be reset.
   * @param nCols the number of columns to reset.
   */
  void clearCols(unsigned int index, unsigned int nCols);

  /*!
   * Clear a number of rows of the grid map.
   * @param index the upper index for the rows to be reset.
   * @param nRows the number of rows to reset.
   */
  void clearRows(unsigned int index, unsigned int nRows);

  /*!
   * Resize the buffer.
   * @param bufferSize the requested buffer size.
   */
  void resizeBuffer(const Eigen::Array2i& bufferSize);

  //! Frame id of the grid map.
  std::string frameId_;

  //! Timestamp of the grid map (nanoseconds).
  uint64_t timestamp_;

  //! Grid map data as matrix.
  std::unordered_map<std::string, Eigen::MatrixXf> data_;

  //! Definition/description of the data types.
  std::vector<std::string> types_;

  //! List of types that are required to be set to NAN when clearing cells of the map.
  std::vector<std::string> clearTypes_;

  //! Side length of the map in x- and y-direction [m].
  Eigen::Array2d length_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;

  //! Map position in the grid map frame [m].
  Eigen::Vector2d position_;

  //! Size of the buffer (rows and cols of the data structure).
  Eigen::Array2i bufferSize_;

  //! Circular buffer start indeces.
  Eigen::Array2i bufferStartIndex_;
};

} /* namespace */
