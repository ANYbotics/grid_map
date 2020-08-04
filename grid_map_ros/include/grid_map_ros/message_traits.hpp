#ifndef GRID_MAP_ROS__MESSAGE_TRAITS_HPP_
#define GRID_MAP_ROS__MESSAGE_TRAITS_HPP_

#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <type_traits>
#include <memory>

namespace ros
{
namespace message_traits
{

template<typename M, typename = void>
struct HasHeader : public std::false_type {};

template<typename M>
struct HasHeader<M, decltype((void) M::header)>: std::true_type {};

template<typename M, typename Enable = void>
struct Header
{
  static std_msgs::msg::Header * pointer(M & m)
  {
    (void)m;
    return nullptr;
  }
  static std_msgs::msg::Header const * pointer(const M & m)
  {
    (void)m;
    return nullptr;
  }
};

template<typename M>
struct Header<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  static std_msgs::msg::Header * pointer(M & m)
  {
    return &m.header;
  }

  static std_msgs::msg::Header const * pointer(const M & m)
  {
    return &m.header;
  }
};

template<typename M, typename Enable = void>
struct FrameId
{
  static std::string * pointer(M & m)
  {
    (void)m;
    return nullptr;
  }
  static std::string const * pointer(const M & m)
  {
    (void)m;
    return nullptr;
  }
};

template<typename M>
struct FrameId<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  static std::string * pointer(M & m)
  {
    return &m.header.frame_id;
  }
  static std::string const * pointer(const M & m)
  {
    return &m.header.frame_id;
  }
  static std::string value(const M & m)
  {
    return m.header.frame_id;
  }
};

template<typename M, typename Enable = void>
struct TimeStamp
{
  static std::unique_ptr<rclcpp::Time> pointer(const M & m)
  {
    (void)m;
    return nullptr;
  }

  static rclcpp::Time value(const M & m)
  {
    (void)m;
    return rclcpp::Time();
  }
};

template<typename M>
struct TimeStamp<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  static std::unique_ptr<rclcpp::Time> pointer(const M & m)
  {
    auto stamp = m.header.stamp;
    return std::make_unique<rclcpp::Time>(rclcpp::Time(stamp.sec, stamp.nanosec));
  }

  static rclcpp::Time value(const M & m)
  {
    auto stamp = m.header.stamp;
    return rclcpp::Time(stamp.sec, stamp.nanosec);
  }
};

}  // namespace message_traits
}  // namespace ros
#endif  // GRID_MAP_ROS__MESSAGE_TRAITS_HPP_
