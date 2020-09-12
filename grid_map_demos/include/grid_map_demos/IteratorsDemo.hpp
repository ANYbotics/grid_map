/*
 * IteratorsDemo.hpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#ifndef GRID_MAP_DEMOS__ITERATORSDEMO_HPP_
#define GRID_MAP_DEMOS__ITERATORSDEMO_HPP_

#include <grid_map_ros/grid_map_ros.hpp>

// ROS
#include <ros/ros.h>

namespace grid_map_demos
{

/*!
 * Visualizes a grid map by publishing different topics that can be viewed in Rviz.
 */
class IteratorsDemo
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  IteratorsDemo(ros::NodeHandle & nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~IteratorsDemo();

  /*!
   * Demos.
   */
  void demoGridMapIterator();
  void demoSubmapIterator();
  void demoCircleIterator();
  void demoEllipseIterator();
  void demoSpiralIterator();
  void demoLineIterator();
  void demoPolygonIterator(const bool prepareForOtherDemos = false);
  void demoSlidingWindowIterator();

  /*!
   * Publish the grid map to ROS.
   */
  void publish();

private:
  //! ROS nodehandle.
  ros::NodeHandle & nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Polygon publisher.
  ros::Publisher polygonPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;
};

}  // namespace grid_map_demos
#endif  // GRID_MAP_DEMOS__ITERATORSDEMO_HPP_
