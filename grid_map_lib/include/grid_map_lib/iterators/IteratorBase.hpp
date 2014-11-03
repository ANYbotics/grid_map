/*
 * IteratorBase.hpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_lib/GridMap.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map_lib {

/*!
 * Iterator class to iterate trough the entire grid map.
 */
class IteratorBase
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   */
  IteratorBase(const grid_map_lib::GridMap &gridMap);

  /*!
   * Copy constructor.
   * @param other the object to copy.
   */
  IteratorBase(const IteratorBase* other);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  IteratorBase& operator =(const IteratorBase& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const IteratorBase& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Eigen::Array2i& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  IteratorBase& operator ++();

  /*!
   * Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++).
   */
  IteratorBase end() const;

  /*!
   * Indicates if iterator is passed end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPassedEnd() const;

private:

  //! Size of the buffer.
  Eigen::Array2i bufferSize_;

  //! Start index of the circular buffer.
  Eigen::Array2i startIndex_;

  //! End index of the circular buffer.
  Eigen::Array2i endIndex_;

  //! Current index.
  Eigen::Array2i index_;

  //! Is iterator out of scope.
  bool isPassedEnd_;
};

} /* namespace */
