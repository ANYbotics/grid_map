#ifndef TEST_COSTMAP_2D_ROS_HPP_
#define TEST_COSTMAP_2D_ROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <atomic>
#include <map>
#include <string>
#include <thread>
#include <memory>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_costmap_2d/grid_map_costmap_2d.hpp"

/*****************************************************************************
** Transforms
*****************************************************************************/

/**
 * @brief Broadcast a set of transforms useful for various demos.
 */
class TransformBroadcaster
{
public:
  explicit TransformBroadcaster(std::shared_ptr<rclcpp::Node> & node)
  : shutdownFlag_(false),
    tfBroadcaster_(*node)
  {}
  virtual ~TransformBroadcaster();
  void add(
    const std::string & name,
    tf2::Vector3 origin,
    const tf2::Quaternion & orientation);

  void startBroadCastingThread();
  void broadcast();
  void shutdown();

private:
  std::map<std::string, geometry_msgs::msg::Transform> transforms_;
  std::thread broadcastingThread_;
  std::atomic<bool> shutdownFlag_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
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
class ROSCostmapServer
{
public:
  typedef nav2_costmap_2d::Costmap2DROS ROSCostmap;
  typedef std::shared_ptr<ROSCostmap> ROSCostmapPtr;

  ROSCostmapServer(
    const std::string & name,
    const std::string & baseLinkTransformName,
    const grid_map::Position & origin,
    const int & width,
    const int & height);

  ~ROSCostmapServer()
  {
    costmap_->deactivate();
    costmap_->shutdown();
  }

  ROSCostmapPtr getROSCostmap()
  {
    return costmap_;
  }

private:
  ROSCostmapPtr costmap_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
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
void broadcastCostmap2DROSTestSuiteTransforms(TransformBroadcaster & broadcaster);

#endif  // TEST_COSTMAP_2D_ROS_HPP_
