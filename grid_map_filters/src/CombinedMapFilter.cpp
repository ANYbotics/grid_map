/*
 * CombinedMapFilter.cpp
 */

#include "grid_map_filters/CombinedMapFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>
#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
CombinedMapFilter<T>::CombinedMapFilter()
: we_(1.0),
  wr_(0.0)
{
}

// template<typename T>
// CombinedMapFilter<T>::~CombinedMapFilter()
// {
//   if (running_) {
//     executor_.cancel();
//     if (spinThread_.joinable()) {
//       spinThread_.join();
//     }
//   }
// }

template<typename T>
CombinedMapFilter<T>::~CombinedMapFilter()
{
}

template<typename T>
bool CombinedMapFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (!param_reader.get("input_layers", inputLayers_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "CombinedMapFilter: missing parameter 'input_layers'.");
    return false;
  }

  if (inputLayers_.size() != 2) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "CombinedMapFilter requires exactly two input layers.");
    return false;
  }

  if (!param_reader.get("output_layer", outputLayer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "CombinedMapFilter: missing parameter 'output_layer'.");
    return false;
  }

  // Set up an internal node purely to listen to /clock.
  clockNode_ = std::make_shared<rclcpp::Node>("combined_map_filter_clock_listener");

  clockSub_ = clockNode_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS(),
    [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
      double t = static_cast<double>(msg->clock.sec) +
        static_cast<double>(msg->clock.nanosec) * 1e-9;
      currentSimTime_.store(t);
    });

  executor_.add_node(clockNode_);
  running_ = true;
  spinThread_ = std::thread([this]() {
      executor_.spin();
    });

  return true;
}

template<typename T>
bool CombinedMapFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;

  double t = currentSimTime_.load();

  // Guard against divide-by-zero at sim start (sec == 0).
  we_ =  t/900; //(t > 1e-6) ? std::min(1.0, 1.0 / t) : 1.0;
  // 900 for mission time for now
  
  wr_ = 1.0 - 2*we_;

  RCLCPP_INFO(
    this->logging_interface_->get_logger(),
    "CombinedMapFilter: t=%.3f, we=%.3f, wr=%.3f", t, we_, wr_);

  const auto & energy = mapOut[inputLayers_[0]];
  const auto & risk = mapOut[inputLayers_[1]];

  Eigen::MatrixXf combined =
    static_cast<float>(we_) * energy +
    static_cast<float>(wr_) * risk;

  mapOut.add(outputLayer_, combined);

  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::CombinedMapFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)