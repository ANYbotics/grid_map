/*
 * GridMapMsgHelpers.hpp
 *
 *  Created on: Sep 8, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>

// Eigen
#include <Eigen/Core>

namespace grid_map {

/*!
 * Returns the number of dimensions of the grid map.
 * @return number of dimensions.
 */
const int nDimensions();

enum class StorageIndices {
    Column,
    Row
};

//! Holds the names of the storage indeces.
extern std::map<StorageIndices, std::string> storageIndexNames;

/*!
 * Checks if message data is stored in row-major format.
 * @param[in] messageData the message data.
 * @return true if is in row-major format, false if is in column-major format.
 */
bool isRowMajor(const std_msgs::Float32MultiArray& messageData);

/*!
 * Returns the number of columns of the message data.
 * @param[in] messageData the message data.
 * @return the number of columns.
 */
unsigned int getCols(const std_msgs::Float32MultiArray& messageData);

/*!
 * Returns the number of rows of the message data.
 * @param[in] messageData the message data.
 * @return the number of rows.
 */
unsigned int getRows(const std_msgs::Float32MultiArray& messageData);

/*!
 * Copies an Eigen matrix into a ROS MultiArray message.
 * Both column- and row-major matrices are allowed, and the type
 * will be marked in the layout labels.
 * @param[in] e the Eigen matrix to be converted.
 * @param[out] m the ROS message to which the data will be copied.
 * @return true if successful
 */
template<typename EigenType_, typename MessageType_>
bool matrixEigenCopyToMultiArrayMessage(const EigenType_& e, MessageType_& m)
{
  m.layout.dim.resize(nDimensions());
  m.layout.dim[0].stride = e.size();
  m.layout.dim[0].size = e.outerSize();
  m.layout.dim[1].stride = e.innerSize();
  m.layout.dim[1].size = e.innerSize();

  if (e.IsRowMajor) {
    m.layout.dim[0].label = storageIndexNames[StorageIndices::Row];
    m.layout.dim[1].label = storageIndexNames[StorageIndices::Column];
  } else {
    m.layout.dim[0].label = storageIndexNames[StorageIndices::Column];
    m.layout.dim[1].label = storageIndexNames[StorageIndices::Row];
  }

  m.data.insert(m.data.begin() + m.layout.data_offset, e.data(), e.data() + e.size());

  return true;
}

/*!
 * Copies a ROS Float64MultiArray message into an Eigen matrix.
 * Both column- and row-major message types are allowed.
 * @param[in] m the ROS message to be converted.
 * @param[out] e the Eigen matrix to which the data will be copied.
 * @return true if successful
 */
bool multiArrayMessageCopyToMatrixEigen(const std_msgs::Float32MultiArray& m, Eigen::MatrixXf& e);

/*!
 * Maps a ROS Float64MultiArray message into an Eigen matrix.
 * Both column- and row-major message types are allowed.
 * @param[in] m the ROS message to be converted.
 * @param[out] e the Eigen matrix to which the data will be mapped.
 * @return true if successful
 */
bool multiArrayMessageMapToMatrixEigen(std_msgs::Float32MultiArray& m, Eigen::MatrixXf& e);

/*!
 * Transforms an int color value (concatenated RGB values) to an int color vector (RGB from 0-255).
 * @param [in] colorValue the concatenated RGB color value.
 * @param [out] colorVector the color vector in RGB from 0-255.
 * @return true if successful.
 */
bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3i& colorVector);

/*!
 * Transforms an int color value (concatenated RGB values) to a float color vector (RGB from 0.0-1.0).
 * @param [in] colorValue the concatenated RGB color value.
 * @param [out] colorVector the color vector in RGB from 0.0-1.0.
 * @return true if successful.
 */
bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3f& colorVector);

/*!
 * Transforms an int color vector (RGB from 0-255) to a concatenated RGB int color.
 * @param [in] colorVector the color vector in RGB from 0-255.
 * @param [out] colorValue the concatenated RGB color value.
 * @return true if successful.
 */
bool colorVectorToValue(const Eigen::Vector3i& colorVector, unsigned long& colorValue);

/*!
 * Transforms a color vector (RGB from 0-255) to a concatenated 3 single-byte float value.
 * @param [in] colorVector the color vector in RGB from 0-255.
 * @param [out] colorValue the concatenated RGB color value.
 */
void colorVectorToValue(const Eigen::Vector3i& colorVector, float& colorValue);

} /* namespace */
