/**
 * @file /grid_map_ros/rostest/costmap_2d_ros/costmap_2d_ros.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef grid_map_ros_ROSTEST_COSTMAP_2D_ROS_HPP_
#define grid_map_ros_ROSTEST_COSTMAP_2D_ROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <atomic>
#include <costmap_2d/costmap_2d_ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <map>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>

/*****************************************************************************
** Transforms
*****************************************************************************/

/**
 * @brief Broadcast a set of transforms useful for various demos.
 */
class TransformBroadcaster {
public:
  TransformBroadcaster() : shutdown_flag(false) {}
  virtual ~TransformBroadcaster();
  void add(const std::string& name, tf::Vector3 origin, const tf::Quaternion& orientation);

  void startBroadCastingThread();
  void broadcast();
  void shutdown();

private:
  std::map<std::string, tf::Transform> transforms;
  std::thread broadcasting_thread;
  std::atomic<bool> shutdown_flag;
};

/**
 * Some partial customisation of various ros costmaps for use with
 * converter demos and tests.
 *
 * Characteristics/Constraints:
 *
 * - fills with stripes, each with an increasing cost value
 * - second last stripe is filled with LETHAL_OBSTACLE cost
 * - last stripe is filled with NO_INFORMATION cost
 */
class ROSCostmapServer {
public:
  typedef costmap_2d::Costmap2DROS ROSCostmap;
  typedef std::shared_ptr<ROSCostmap> ROSCostmapPtr;

  ROSCostmapServer(const std::string& name,
                   const std::string& base_link_transform_name,
                   const grid_map::Position& origin,
                   const double& width,
                   const double& height
  );

  ROSCostmapPtr getROSCostmap() { return costmap; };

private:
  ROSCostmapPtr costmap;
  tf::TransformListener transform_listener;
};


/*****************************************************************************
** Helpers
*****************************************************************************/

/**
 * Broadcast a set of transform useful for the suite of Costmap2DROS
 * converter demos and tests
 *
 * @param[in] broadcaster : uninitialised broadcaster object
 */
void broadcastCostmap2DROSTestSuiteTransforms(TransformBroadcaster& broadcaster);


#endif /* grid_map_ros_ROSTEST_COSTMAP_2D_ROS_HPP_ */
