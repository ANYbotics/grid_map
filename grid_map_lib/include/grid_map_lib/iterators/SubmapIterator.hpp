/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 12, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_lib/GridMap.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map_lib {

/*!
 * Iterator class to iterate through a rectangular part of the map (submap).
 * Before using this iterator, make sure that the requested submap is
 * actually contained in the grid map.
 */
class SubmapIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param submapStartIndex the start index of the submap, typically top-left index.
   * @param submapBufferSize the size of the submap to iterate on.
   */
  SubmapIterator(const grid_map_lib::GridMap& gridMap,
                 const Eigen::Array2i& submapStartIndex,
                 const Eigen::Array2i& submapBufferSize);
  /*!
   * Copy constructor.
   * @param other the object to copy.
   */
  SubmapIterator(const SubmapIterator* other);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  SubmapIterator& operator =(const SubmapIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const SubmapIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Eigen::Array2i& operator *() const;

  /*!
   * Get the current index in the submap.
   * @return the current index in the submap.
   */
  const Eigen::Array2i& getSubmapIndex() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  SubmapIterator& operator ++();

  /*!
   * Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++).
   */
  SubmapIterator end() const;

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

  //! Current index.
  Eigen::Array2i index_;

  //! Submap buffer size.
  Eigen::Array2i submapBufferSize_;

  //! Top left index of the submap.
  Eigen::Array2i submapStartIndex_;

  //! End index of the submap.
  Eigen::Array2i submapEndIndex_;

  //! Current index in the submap.
  Eigen::Array2i submapIndex_;

  //! Is iterator out of scope.
  bool isPassedEnd_;
};

} /* namespace */
