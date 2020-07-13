/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"

#include "test_costmap_2d_ros.hpp"

/*****************************************************************************
** Global rclcpp::Node
*****************************************************************************/
std::shared_ptr<rclcpp::Node> node_ = nullptr;

/*****************************************************************************
** Helpers
*****************************************************************************/

void broadcastCostmap2DROSTestSuiteTransforms(TransformBroadcaster & broadcaster)
{
  broadcaster.add(
    "base_link_5x5",
    tf2::Vector3(1.0, 1.0, 0.0),
    tf2::Quaternion(0, 0, 0, 1));
  broadcaster.add(
    "base_link_4x4",
    tf2::Vector3(1.0, -3.0, 0.0),
    tf2::Quaternion(0, 0, 0, 1));
  broadcaster.add(
    "base_link_5x5_3x3_offset",
    tf2::Vector3(-3.7, 2.4, 0.0),
    tf2::Quaternion(0, 0, 0, 1));
  broadcaster.add(
    "base_link_5x5_3x3_centre",
    tf2::Vector3(-3.5, -3.5, 0.0),
    tf2::Quaternion(0, 0, 0, 1));
  broadcaster.add(
    "base_link_5x5_2_5x2_5_offset",
    tf2::Vector3(-9.7, 2.4, 0.0),
    tf2::Quaternion(0, 0, 0, 1));
  broadcaster.startBroadCastingThread();
}

/*****************************************************************************
** ROS Costmap Server
*****************************************************************************/

ROSCostmapServer::ROSCostmapServer(
  const std::string & name,
  const std::string & baseLinkTransformName,
  const grid_map::Position & origin,
  const int & width,
  const int & height)
: tfBuffer_(node_->get_clock()),
  tfListener_(tfBuffer_)
{
  // lots of parameters here affect the construction ( e.g. rolling window)
  // if you don't have any parameters set, then this
  //  - alot of defaults which get dumped on the ros param server
  //  - fires up an obstacle layer and an inflation layer
  //  - creates a publisher for an occupancy grid

  costmap_ = std::make_shared<ROSCostmap>(name);
  costmap_->set_parameter(rclcpp::Parameter{"plugins", std::vector<std::string>()});
  costmap_->set_parameter(rclcpp::Parameter{"robot_base_frame", baseLinkTransformName});
  costmap_->set_parameter(rclcpp::Parameter{"origin_x", origin.x()});
  costmap_->set_parameter(rclcpp::Parameter{"origin_y", origin.y()});
  costmap_->set_parameter(rclcpp::Parameter{"width", width});
  costmap_->set_parameter(rclcpp::Parameter{"height", height});
  costmap_->set_parameter(rclcpp::Parameter{"resolution", 0.5});
  // clears 1 cell if inside, up to 4 cells on a vertex
  costmap_->set_parameter(rclcpp::Parameter{"robot_radius", 0.03});

  costmap_->configure();
  costmap_->activate();

  for (unsigned int index = 0; index < costmap_->getCostmap()->getSizeInCellsY(); ++index) {
    unsigned int dimension = costmap_->getCostmap()->getSizeInCellsX();
    // @todo assert dimension > 1
    // set the first row to nav2_costmap_2d::FREE_SPACE?
    // but it shows up invisibly in rviz, so don't bother
    for (unsigned int fill_index = 0; fill_index < dimension - 2; ++fill_index) {
      double fraction = static_cast<double>(fill_index + 1) /
        static_cast<double>(costmap_->getCostmap()->getSizeInCellsX());
      costmap_->getCostmap()->setCost(
        fill_index,
        index,
        fraction * nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
    costmap_->getCostmap()->setCost(dimension - 2, index, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap_->getCostmap()->setCost(dimension - 1, index, nav2_costmap_2d::NO_INFORMATION);
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

void TransformBroadcaster::add(
  const std::string & name,
  tf2::Vector3 origin,
  const tf2::Quaternion & orientation)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = origin.x();
  transform.translation.y = origin.y();
  transform.translation.z = origin.z();
  transform.rotation.x = orientation.x();
  transform.rotation.y = orientation.y();
  transform.rotation.z = orientation.z();
  transform.rotation.w = orientation.w();
  transforms_.insert(std::pair<std::string, geometry_msgs::msg::Transform>(name, transform));
}

void TransformBroadcaster::startBroadCastingThread()
{
  broadcastingThread_ = std::thread(&TransformBroadcaster::broadcast, this);
}

void TransformBroadcaster::broadcast()
{
  while (rclcpp::ok() && !shutdownFlag_) {
    for (std::pair<std::string, geometry_msgs::msg::Transform> p : transforms_) {
      geometry_msgs::msg::TransformStamped transformMsg;
      transformMsg.header.stamp = node_->now();
      transformMsg.header.frame_id = "map";
      transformMsg.child_frame_id = p.first;
      transformMsg.transform = p.second;
      tfBroadcaster_.sendTransform(transformMsg);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

/*****************************************************************************
** Converter Functions
*****************************************************************************/

bool fromCostmap2DROS(
  nav2_costmap_2d::Costmap2DROS & ros_costmap,
  const std::string & layer_name,
  grid_map::GridMap & grid_map)
{
  grid_map::Costmap2DConverter<grid_map::GridMap> converter;
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *(ros_costmap.getCostmap()->getMutex()));
  converter.initializeFromCostmap2D(ros_costmap, grid_map);
  return converter.addLayerFromCostmap2D(ros_costmap, layer_name, grid_map);
}

bool fromCostmap2DROSAtRobotPose(
  nav2_costmap_2d::Costmap2DROS & ros_costmap,
  const grid_map::Length & geometry,
  const std::string & layer_name,
  grid_map::GridMap & grid_map)
{
  grid_map::Costmap2DConverter<grid_map::GridMap> converter;
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *(ros_costmap.getCostmap()->getMutex()));
  if (!converter.initializeFromCostmap2DAtRobotPose(ros_costmap, geometry, grid_map)) {
    return false;
  }
  return converter.addLayerFromCostmap2DAtRobotPose(ros_costmap, layer_name, grid_map);
}

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Costmap2DROSConversion, full_window)
{
  std::cout << std::endl;
  RCLCPP_INFO(node_->get_logger(), "***********************************************************");
  RCLCPP_INFO(node_->get_logger(), "                 Copy Full Window");
  RCLCPP_INFO(node_->get_logger(), "***********************************************************");
  // preparation
  std::string layer_name = "obstacle_costs";
  ROSCostmapServer
    ros_costmap_5x5("five_by_five", "base_link_5x5", grid_map::Position(0.0, 0.0), 5.0, 5.0);
  grid_map::GridMap grid_map_5x5;
  fromCostmap2DROS(*(ros_costmap_5x5.getROSCostmap()), layer_name, grid_map_5x5);
  // assert map properties
  ASSERT_EQ(grid_map_5x5.getFrameId(), ros_costmap_5x5.getROSCostmap()->getGlobalFrameID());
  ASSERT_EQ(
    grid_map_5x5.getLength().x(),
    ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsX() *
    ros_costmap_5x5.getROSCostmap()->getCostmap()->getResolution());
  ASSERT_EQ(
    grid_map_5x5.getLength().y(),
    ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsY() *
    ros_costmap_5x5.getROSCostmap()->getCostmap()->getResolution());
  ASSERT_EQ(
    (unsigned int) grid_map_5x5.getSize()[0],
    ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsX());
  ASSERT_EQ(
    (unsigned int) grid_map_5x5.getSize()[1],
    ros_costmap_5x5.getROSCostmap()->getCostmap()->getSizeInCellsY());
  grid_map::Length position = grid_map_5x5.getPosition() - 0.5 * grid_map_5x5.getLength().matrix();
  ASSERT_EQ(position.x(), ros_costmap_5x5.getROSCostmap()->getCostmap()->getOriginX());
  ASSERT_EQ(position.y(), ros_costmap_5x5.getROSCostmap()->getCostmap()->getOriginY());

  // assert map data
  for (unsigned int i = 0; i < 5; ++i) {
    for (unsigned int j = 0; j < 5; ++j) {
      std::cout <<
        static_cast<int>(ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(i, j)) << " ";
    }
    std::cout << std::endl;
  }
  for (unsigned int i = 0; i < 5; ++i) {
    for (unsigned int j = 0; j < 5; ++j) {
      std::cout << static_cast<int>(grid_map_5x5.at(layer_name, grid_map::Index(i, j))) << " ";
    }
    std::cout << std::endl;
  }
  // TODO(tbd): a function which does the index conversion

  std::cout << "Original cost: " <<
    static_cast<int>(ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(8, 0)) <<
    std::endl;
  std::cout << "New Cost: " << grid_map_5x5.at(layer_name, grid_map::Index(1, 9)) << std::endl;
  std::vector<float> cost_translation_table;
  grid_map::Costmap2DCenturyTranslationTable::create(cost_translation_table);
  unsigned char cost = ros_costmap_5x5.getROSCostmap()->getCostmap()->getCost(8, 0);
  ASSERT_EQ(grid_map_5x5.at(layer_name, grid_map::Index(1, 9)), cost_translation_table[cost]);
  std::cout << std::endl;
}

TEST(Costmap2DROSConversion, cost_map_centres)
{
  std::cout << std::endl;
  RCLCPP_INFO(node_->get_logger(), "***********************************************************");
  RCLCPP_INFO(node_->get_logger(), "                 Check Subwindow Centres");
  RCLCPP_INFO(node_->get_logger(), "***********************************************************");
  RCLCPP_INFO(node_->get_logger(), "Subwindows are centred as closely as possible to the robot");
  RCLCPP_INFO(node_->get_logger(), "pose, though not exactly. They still need to align with");
  RCLCPP_INFO(node_->get_logger(), "the underlying ros costmap so that they don't introduce a");
  RCLCPP_INFO(node_->get_logger(), "new kind of error. As a result, the centre is shifted from");
  RCLCPP_INFO(node_->get_logger(), "the robot pose to the nearest appropriate point which aligns");
  RCLCPP_INFO(node_->get_logger(), "the new cost map exactly on top of the original ros costmap.");
  std::cout << std::endl;
  std::string layer_name = "obstacle_costs";
  ROSCostmapServer ros_costmap_5x5_3x3_offset(
    "five_by_five_three_by_three_offset",
    "base_link_5x5_3x3_offset",
    grid_map::Position(-6.0, 0.0),
    5,
    5);
  ROSCostmapServer ros_costmap_5x5_3x3_centre(
    "five_by_five_three_by_three_centre",
    "base_link_5x5_3x3_centre",
    grid_map::Position(-6.0, -6.0),
    5,
    5);
  ROSCostmapServer ros_costmap_5x5_2_5x2_5_offset(
    "five_by_five_twohalf_by_twohalf_offset",
    "base_link_5x5_2_5x2_5_offset",
    grid_map::Position(-12.0, 0.0),
    5,
    5);
  grid_map::GridMap grid_map_5x5_3x3_offset, grid_map_5x5_3x3_centre, grid_map_5x5_2_5x2_5_offset;
  grid_map::Length geometry_3x3(3.0, 3.0);
  fromCostmap2DROSAtRobotPose(
    *(ros_costmap_5x5_3x3_offset.getROSCostmap()),
    geometry_3x3,
    layer_name,
    grid_map_5x5_3x3_offset);
  fromCostmap2DROSAtRobotPose(
    *(ros_costmap_5x5_3x3_centre.getROSCostmap()),
    geometry_3x3,
    layer_name,
    grid_map_5x5_3x3_centre);
  grid_map::Length geometry_2_5x2_5(2.5, 2.5);
  fromCostmap2DROSAtRobotPose(
    *(ros_costmap_5x5_2_5x2_5_offset.getROSCostmap()),
    geometry_2_5x2_5,
    layer_name,
    grid_map_5x5_2_5x2_5_offset);
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "  grid_map_5x5_3x3_offset : " << grid_map_5x5_3x3_offset.getPosition().transpose());
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "  grid_map_5x5_3x3_offset : " << grid_map_5x5_3x3_centre.getPosition().transpose());
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "  grid_map_5x5_3x3_offset : " << grid_map_5x5_2_5x2_5_offset.getPosition().transpose());
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("test_from_ros_costmaps");

  TransformBroadcaster broadcaster(node_);
  broadcastCostmap2DROSTestSuiteTransforms(broadcaster);

  testing::InitGoogleTest(&argc, argv);

  int test_result = RUN_ALL_TESTS();
  RCLCPP_INFO(node_->get_logger(), "gtest return value: %d", test_result);

  broadcaster.shutdown();
  rclcpp::shutdown();

  node_.reset();

  return test_result;
}
