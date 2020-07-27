#ifndef GRID_MAP_ROS__MESSAGE_TRAITS_HPP_
#define GRID_MAP_ROS__MESSAGE_TRAITS_HPP_

#include <grid_map_msgs/GridMap.h>

#include <string>

namespace ros
{
namespace message_traits
{

template<>
struct HasHeader<grid_map_msgs::GridMap>: public TrueType {};

template<>
struct Header<grid_map_msgs::GridMap,
  typename boost::enable_if<HasHeader<grid_map_msgs::GridMap>>::type>
{
  static std_msgs::Header * pointer(grid_map_msgs::GridMap & m)
  {
    return &m.info.header;
  }

  static std_msgs::Header const * pointer(const grid_map_msgs::GridMap & m)
  {
    return &m.info.header;
  }
};

template<>
struct FrameId<grid_map_msgs::GridMap,
  typename boost::enable_if<HasHeader<grid_map_msgs::GridMap>>::type>
{
  static std::string * pointer(grid_map_msgs::GridMap & m)
  {
    return &m.info.header.frame_id;
  }

  static std::string const * pointer(const grid_map_msgs::GridMap & m)
  {
    return &m.info.header.frame_id;
  }

  static std::string value(const grid_map_msgs::GridMap & m)
  {
    return m.info.header.frame_id;
  }
};

template<>
struct TimeStamp<grid_map_msgs::GridMap,
  typename boost::enable_if<HasHeader<grid_map_msgs::GridMap>>::type>
{
  static rclcpp::Time * pointer(grid_map_msgs::GridMap & m)
  {
    return &m.info.header.stamp;
  }

  static rclcpp::Time const * pointer(const grid_map_msgs::GridMap & m)
  {
    return &m.info.header.stamp;
  }

  static rclcpp::Time value(const grid_map_msgs::GridMap & m)
  {
    return m.info.header.stamp;
  }
};

}  // namespace message_traits
}  // namespace ros
#endif  // GRID_MAP_ROS__MESSAGE_TRAITS_HPP_
