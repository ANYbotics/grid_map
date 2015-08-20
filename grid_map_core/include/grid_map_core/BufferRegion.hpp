/*
 * BufferRegion.hpp
 *
 *  Created on: Aug 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "grid_map_core/TypeDefs.hpp"

namespace grid_map {

class BufferRegion
{
 public:

  /*!
   * The definition of the buffer region positions.
   */
  enum class Quadrant
  {
    Undefined,
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
  };

  constexpr static unsigned int nQuadrants = 4;

  BufferRegion();
  BufferRegion(const Index& index, const Size& size, const BufferRegion::Quadrant& quadrant);
  virtual ~BufferRegion();

  const Index& getIndex() const;
  void setIndex(const Index& index);
  const Size& getSize() const;
  void setSize(const Size& size);
  BufferRegion::Quadrant getQuadrant() const;
  void setQuadrant(BufferRegion::Quadrant type);

 private:

  //! Top left index of the buffer region.
  Index index_;

  //! Size of the buffer region.
  Size size_;

  //! Quadrant type of the buffer region.
  Quadrant quadrant_;
};

} /* namespace grid_map */
