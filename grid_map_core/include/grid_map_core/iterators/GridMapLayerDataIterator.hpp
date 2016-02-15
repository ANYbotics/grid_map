/*
 * GridMapLayerDataIterator.hpp
 *
 *  Created on: Feb 15, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/GridMap.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map {

/*!
 * Iterator class to iterate trough the entire grid map.
 */
class GridMapLayerDataIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param layer the layer to take the data from.
   */
  GridMapLayerDataIterator(const grid_map::GridMap &gridMap, const std::string& layer);

  /*!
   * Copy constructor.
   * @param other the object to copy.
   */
  GridMapLayerDataIterator(const GridMapLayerDataIterator* other);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  GridMapLayerDataIterator& operator =(const GridMapLayerDataIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const GridMapLayerDataIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  Index operator *() const;

  /*!
   * Returns the stored value of the cell to which the iterator is pointing to.
   * @return the cell's value.
   */
  const float& getValue() const;

  /*!
   * Returns the the linear (1-dim.) index of the cell the iterator is pointing to.
   * @return the 1d linear index.
   */
  const size_t& getLinearIndex() const;

  /*!
   * Retrieve the index as unwrapped index, i.e., as the corresponding index of a
   * grid map with no circular buffer offset.
   */
  const Index getUnwrappedIndex() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  GridMapLayerDataIterator& operator ++();

  /*!
   * Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++).
   */
  GridMapLayerDataIterator end() const;

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  //! Size of the buffer.
  Size size_;

  //! Start index of the circular buffer.
  Index startIndex_;

  //! Linear size of the data.
  size_t linearSize_;

  //! Linear index.
  size_t linearIndex_;

  //! Pointer to the data.
  const Matrix::Scalar* dataStart_;

  //! Is iterator out of scope.
  bool isPastEnd_;
};

} /* namespace */
