/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#if ROS_VERSION_MINIMUM(1,14,0)
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#else
#include <tf/transform_broadcaster.h>
#endif

#include "costmap_2d_ros.hpp"

/*****************************************************************************
** Helpers
*****************************************************************************/

void broadcastCostmap2DROSTestSuiteTransforms(TransformBroadcaster& broadcaster)
{
  broadcaster.add("base_link_5x5", tf::Vector3(1.0, 1.0, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_4x4", tf::Vector3(1.0, -3.0, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_5x5_3x3_offset", tf::Vector3(-3.7, 2.4, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_5x5_3x3_centre", tf::Vector3(-3.5, -3.5, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.add("base_link_5x5_2_5x2_5_offset", tf::Vector3(-9.7, 2.4, 0.0), tf::Quaternion(0, 0, 0, 1));
  broadcaster.startBroadCastingThread();
}

/*****************************************************************************
** ROS Costmap Server
*****************************************************************************/

ROSCostmapServer::ROSCostmapServer(const std::string& name,
                                   const std::string& baseLinkTransformName,
                                   const grid_map::Position& origin, const double& width,
                                   const double& height)
#if ROS_VERSION_MINIMUM(1,14,0)
    : tfBuffer_(ros::Duration(1.0)),
      tfListener_(tfBuffer_)
#else
    : tfListener_(ros::Duration(1.0))
#endif
{
  ros::NodeHandle privateNodeHandle("~");
  // lots of parameters here affect the construction ( e.g. rolling window)
  // if you don't have any parameters set, then this
  //  - alot of defaults which get dumped on the ros param server
  //  - fires up an obstacle layer and an inflation layer
  //  - creates a publisher for an occupancy grid
  privateNodeHandle.setParam(name + "/robot_base_frame", baseLinkTransformName);
  privateNodeHandle.setParam(name + "/origin_x", origin.x());
  privateNodeHandle.setParam(name + "/origin_y", origin.y());
  privateNodeHandle.setParam(name + "/width", width);
  privateNodeHandle.setParam(name + "/height", height);
  privateNodeHandle.setParam(name + "/plugins", std::vector<std::string>());
  privateNodeHandle.setParam(name + "/resolution", 0.5);
  privateNodeHandle.setParam(name + "/robot_radius", 0.03); // clears 1 cell if inside, up to 4 cells on a vertex
#if ROS_VERSION_MINIMUM(1,14,0)
  costmap_ = std::make_shared<ROSCostmap>(name, tfBuffer_);
#else
  costmap_ = std::make_shared<ROSCostmap>(name, tfListener_);
#endif

  for ( unsigned int index = 0; index < costmap_->getCostmap()->getSizeInCellsY(); ++index ) {
    unsigned int dimension = costmap_->getCostmap()->getSizeInCellsX();
    // @todo assert dimension > 1
    // set the first row to costmap_2d::FREE_SPACE? but it shows up invisibly in rviz, so don't bother
    for ( unsigned int fill_index = 0; fill_index < dimension - 2; ++fill_index )
    {
      double fraction = static_cast<double>(fill_index + 1) / static_cast<double>(costmap_->getCostmap()->getSizeInCellsX());
      costmap_->getCostmap()->setCost(fill_index, index, fraction*costmap_2d::INSCRIBED_INFLATED_OBSTACLE );
    }
    costmap_->getCostmap()->setCost(dimension - 2, index, costmap_2d::LETHAL_OBSTACLE);
    costmap_->getCostmap()->setCost(dimension - 1, index, costmap_2d::NO_INFORMATION);
  }
}

/*****************************************************************************
** TransformBroadcaster
*****************************************************************************/

TransformBroadcaster::~TransformBroadcaster()
{
  broadcastingThread_.join();
}

void TransformBroadcaster::shutdown()
{
  shutdownFlag_ = true;
}

void TransformBroadcaster::add(const std::string& name, tf::Vector3 origin,
                               const tf::Quaternion& orientation)
{
  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(orientation);
  transforms_.insert(std::pair<std::string, tf::Transform>(name, transform));
}

void TransformBroadcaster::startBroadCastingThread() {
  broadcastingThread_ = std::thread(&TransformBroadcaster::broadcast, this);
}

void TransformBroadcaster::broadcast()
{
#if ROS_VERSION_MINIMUM(1,14,0)
  tf2_ros::TransformBroadcaster tfBroadcaster;
#else
  tf::TransformBroadcaster tfBroadcaster;
#endif
  while (ros::ok() && !shutdownFlag_) {
    for (std::pair<std::string, tf::Transform> p : transforms_) {
      tf::StampedTransform transform(p.second, ros::Time::now(), "map", p.first);
#if ROS_VERSION_MINIMUM(1,14,0)
      geometry_msgs::TransformStamped transformMsg;
      tf::transformStampedTFToMsg(transform, transformMsg);
      tfBroadcaster.sendTransform(transformMsg);
#else
      tfBroadcaster.sendTransform(transform);
#endif
    }
    ros::Duration(0.1).sleep();
  }
}

/*****************************************************************************
** Converter Functions
*****************************************************************************/

bool fromCostmap2DROS(costmap_2d::Costmap2DROS& ros_costmap, const std::string& layer_name,
                      grid_map::GridMap& grid_map)
{
  grid_map::Costmap2DConverter<grid_map::GridMap> converter;
  boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(ros_costmap.getCostmap()->getMutex()));
  converter.initializeFromCostmap2D(ros_costmap, grid_map);
  if (!converter.addLayerFromCostmap2D(ros_costmap, layer_name, grid_map)) {
    return false;
  }
  return true;
}

bool fromCostmap2DROSAtRobotPose(costmap_2d::Costmap2DROS& ros_costmap,
                                 const grid_map::Length& geometry, const std::string& layer_name,
                                 grid_map::GridMap& grid_map)
{
  grid_map::Costmap2DConverter<grid_map::GridMap> converter;
  boost::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(ros_costmap.getCostmap()->getMutex()));
  if (!converter.initializeFromCostmap2DAtRobotPose(ros_costmap, geometry, grid_map)) {
    return false;
  }
  if (!converter.addLayerFromCostmap2DAtRobotPose(ros_costmap, layer_name, grid_map)) {
    return false;
  }
  return true;
}

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Costmap2DROSConversion, full_window)
{
  std::cout << std::endl;
  ROS_INFO("***********************************************************");
  ROS_INFO("                 Copy Full Window");
  ROS_INFO("***********************************************************");
  // preparation
  std::string layer_name =  "obstacle_costs";
  ROSCostmapServer ros_costmap_5x5("five_by_five", "base_link_5x5", grid_map::Position(0.0, 0.0), 5.0, 5.0);
  grid_map::GridMap grid_map_5x5;
  fromCostmap2DROS(*(ros_costmap_5x5.getROSCostmap()), layer_name, grid_map_5x5);
  // assert map properties
  ASSERT_EQ(grid_map_5x5.getFrameId(), ros_costmap_5x5.getROSCostmap()->getGlobalFrameID());
  ASSERT_EQ(
      grid_map_5x5.getLength().x(),
      ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsX()
          * ros_costmap_5x5.getROSCostmap()->getCostmap()->getResolution());
  ASSERT_EQ(
      grid_map_5x5.getLength().y(),
      ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsY()
          * ros_costmap_5x5.getROSCostmap()->getCostmap()->getResolution());
  ASSERT_EQ(grid_map_5x5.getSize()[0], ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsX());
  ASSERT_EQ(grid_map_5x5.getSize()[1], ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsY());
  grid_map::Length position = grid_map_5x5.getPosition() - 0.5 * grid_map_5x5.getLength().matrix();
  ASSERT_EQ(position.x(), ros_costmap_5x5.getROSCostmap()->getCostmap()->getOriginX());
  ASSERT_EQ(position.y(), ros_costmap_5x5.getROSCostmap()->getCostmap()->getOriginY());

  // assert map data
  for (unsigned int i = 0; i < 5; ++i) {
    for (unsigned int j = 0; j < 5; ++j) {
      std::cout << static_cast<int>(ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(i, j))
                << " ";
    }
    std::cout << std::endl;
  }
  for (unsigned int i = 0; i < 5; ++i) {
    for (unsigned int j = 0; j < 5; ++j) {
      std::cout << static_cast<int>(grid_map_5x5.at(layer_name, grid_map::Index(i, j))) << " ";
    }
    std::cout << std::endl;
  }
  // TODO a function which does the index conversion

  std::cout << "Original cost: " << static_cast<int>(ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(8,0)) << std::endl;
  std::cout << "New Cost: " << grid_map_5x5.at(layer_name, grid_map::Index(1,9)) << std::endl;
  std::vector<float> cost_translation_table;
  grid_map::Costmap2DCenturyTranslationTable::create(cost_translation_table);
  unsigned char cost = ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(8,0);
  ASSERT_EQ(grid_map_5x5.at(layer_name, grid_map::Index(1,9)), cost_translation_table[cost]);
  std::cout << std::endl;
}

TEST(Costmap2DROSConversion, cost_map_centres)
{
  std::cout << std::endl;
  ROS_INFO("***********************************************************");
  ROS_INFO("                 Check Subwindow Centres");
  ROS_INFO("***********************************************************");
  ROS_INFO("Subwindows are centred as closely as possible to the robot");
  ROS_INFO("pose, though not exactly. They still need to align with");
  ROS_INFO("the underlying ros costmap so that they don't introduce a");
  ROS_INFO("new kind of error. As a result, the centre is shifted from");
  ROS_INFO("the robot pose to the nearest appropriate point which aligns");
  ROS_INFO("the new cost map exactly on top of the original ros costmap.");
  std::cout << std::endl;
  std::string layer_name =  "obstacle_costs";
  ROSCostmapServer ros_costmap_5x5_3x3_offset("five_by_five_three_by_three_offset", "base_link_5x5_3x3_offset", grid_map::Position(-6.0, 0.0), 5.0, 5.0);
  ROSCostmapServer ros_costmap_5x5_3x3_centre("five_by_five_three_by_three_centre", "base_link_5x5_3x3_centre", grid_map::Position(-6.0, -6.0), 5.0, 5.0);
  ROSCostmapServer ros_costmap_5x5_2_5x2_5_offset("five_by_five_twohalf_by_twohalf_offset", "base_link_5x5_2_5x2_5_offset", grid_map::Position(-12.0, 0.0), 5.0, 5.0);
  grid_map::GridMap grid_map_5x5_3x3_offset, grid_map_5x5_3x3_centre, grid_map_5x5_2_5x2_5_offset;
  grid_map::Length geometry_3x3(3.0, 3.0);
  fromCostmap2DROSAtRobotPose(*(ros_costmap_5x5_3x3_offset.getROSCostmap()), geometry_3x3, layer_name, grid_map_5x5_3x3_offset);
  fromCostmap2DROSAtRobotPose(*(ros_costmap_5x5_3x3_centre.getROSCostmap()), geometry_3x3, layer_name, grid_map_5x5_3x3_centre);
  grid_map::Length geometry_2_5x2_5(2.5, 2.5);
  fromCostmap2DROSAtRobotPose(*(ros_costmap_5x5_2_5x2_5_offset.getROSCostmap()), geometry_2_5x2_5, layer_name, grid_map_5x5_2_5x2_5_offset);
  ROS_INFO_STREAM("  grid_map_5x5_3x3_offset : " << grid_map_5x5_3x3_offset.getPosition().transpose());
  ROS_INFO_STREAM("  grid_map_5x5_3x3_offset : " << grid_map_5x5_3x3_centre.getPosition().transpose());
  ROS_INFO_STREAM("  grid_map_5x5_3x3_offset : " << grid_map_5x5_2_5x2_5_offset.getPosition().transpose());
  ASSERT_EQ(-3.5, grid_map_5x5_3x3_offset.getPosition().x());
  ASSERT_EQ(2.5, grid_map_5x5_3x3_offset.getPosition().y());
  ASSERT_EQ(-3.5, grid_map_5x5_3x3_centre.getPosition().x());
  ASSERT_EQ(-3.5, grid_map_5x5_3x3_centre.getPosition().y());
  ASSERT_EQ(-9.75, grid_map_5x5_2_5x2_5_offset.getPosition().x());
  ASSERT_EQ(2.25, grid_map_5x5_2_5x2_5_offset.getPosition().y());
  std::cout << std::endl;
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_from_ros_costmaps");

  TransformBroadcaster broadcaster;
  broadcastCostmap2DROSTestSuiteTransforms(broadcaster);

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  broadcaster.shutdown();
  return result;
}
