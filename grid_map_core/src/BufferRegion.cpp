/*
 * BufferRegion.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */
#include <grid_map_core/BufferRegion.hpp>

namespace grid_map {

BufferRegion::BufferRegion() : startIndex_(Index::Zero()),
    size_(Size::Zero()),
    quadrant_(BufferRegion::Quadrant::Undefined)
{
}

BufferRegion::BufferRegion(Index index, Size size, BufferRegion::Quadrant quadrant) : startIndex_(std::move(index)),
    size_(std::move(size)),
    quadrant_(std::move(quadrant))
{
}

const Index& BufferRegion::getStartIndex() const
{
  return startIndex_;
}

void BufferRegion::setStartIndex(const Index& startIndex)
{
  startIndex_ = startIndex;
}

const Size& BufferRegion::getSize() const
{
  return size_;
}

void BufferRegion::setSize(const Size& size)
{
  size_ = size;
}

BufferRegion::Quadrant BufferRegion::getQuadrant() const
{
  return quadrant_;
}

void BufferRegion::setQuadrant(BufferRegion::Quadrant type)
{
  quadrant_ = type;
}

} /* namespace grid_map */


