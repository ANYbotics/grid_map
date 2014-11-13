/*
 * Circleterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/iterators/LineIterator.hpp"
#include "grid_map_lib/GridMapMath.hpp"

using namespace std;

namespace grid_map_lib {

LineIterator::LineIterator(const grid_map_lib::GridMap& gridMap, const Eigen::Vector2d& start, const Eigen::Vector2d& end)
    : start_(start),
      end_(end)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getBufferSize();
  bufferStartIndex_ = gridMap.getBufferStartIndex();
  Eigen::Array2i submapStartIndex;
  Eigen::Array2i submapBufferSize;
//  if(!isInside()) ++(*this);
}

LineIterator& LineIterator::operator =(const LineIterator& other)
{
  start_ = other.start_;
  end_ = other.end_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool LineIterator::operator !=(const LineIterator& other) const
{
//  return (internalIterator_ != other.internalIterator_);
}

const Eigen::Array2i& LineIterator::operator *() const
{
//  return *(*internalIterator_);
}

LineIterator& LineIterator::operator ++()
{
//  ++(*internalIterator_);
//  if (internalIterator_->isPassedEnd()) return *this;
//
//  for ( ; !internalIterator_->isPassedEnd(); ++(*internalIterator_)) {
//    if (isInside()) break;
//  }

  return *this;
}

bool LineIterator::isPassedEnd() const
{
//  return internalIterator_->isPassedEnd();
}

void LineIterator::initializeParameters()
{
  //Bresenham Ray-Tracing
//   int deltax = abs(x1 - x0);        // The difference between the x's
//   int deltay = abs(y1 - y0);        // The difference between the y's
//   int x = x0;                       // Start x off at the first pixel
//   int y = y0;                       // Start y off at the first pixel
//
//   int xinc1, xinc2, yinc1, yinc2;
//   int den, num, numadd, numpixels;
//
//   base_local_planner::Position2DInt pt;
//
//   if (x1 >= x0)                 // The x-values are increasing
//   {
//     xinc1 = 1;
//     xinc2 = 1;
//   }
//   else                          // The x-values are decreasing
//   {
//     xinc1 = -1;
//     xinc2 = -1;
//   }
//
//   if (y1 >= y0)                 // The y-values are increasing
//   {
//     yinc1 = 1;
//     yinc2 = 1;
//   }
//   else                          // The y-values are decreasing
//   {
//     yinc1 = -1;
//     yinc2 = -1;
//   }
//
//   if (deltax >= deltay)         // There is at least one x-value for every y-value
//   {
//     xinc1 = 0;                  // Don't change the x when numerator >= denominator
//     yinc2 = 0;                  // Don't change the y for every iteration
//     den = deltax;
//     num = deltax / 2;
//     numadd = deltay;
//     numpixels = deltax;         // There are more x-values than y-values
//   }
//   else                          // There is at least one y-value for every x-value
//   {
//     xinc2 = 0;                  // Don't change the x for every iteration
//     yinc1 = 0;                  // Don't change the y when numerator >= denominator
//     den = deltay;
//     num = deltay / 2;
//     numadd = deltax;
//     numpixels = deltay;         // There are more y-values than x-values
//   }
//
//   for (int curpixel = 0; curpixel <= numpixels; curpixel++)
//   {
//     pt.x = x;      //Draw the current pixel
//     pt.y = y;
//     pts.push_back(pt);
//
//     num += numadd;              // Increase the numerator by the top of the fraction
//     if (num >= den)             // Check if numerator >= denominator
//     {
//       num -= den;               // Calculate the new numerator value
//       x += xinc1;               // Change the x as appropriate
//       y += yinc1;               // Change the y as appropriate
//     }
//     x += xinc2;                 // Change the x as appropriate
//     y += yinc2;                 // Change the y as appropriate
//   }
}

} /* namespace grid_map_lib */

