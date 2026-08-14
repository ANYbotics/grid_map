/*
 * CombinedMapFilter.hpp
 *
 *  Created on: Nov 28, 2025
 *      Author: Riana Gagnon (based on GridMap filters by ANYbotics)
 */
#ifndef GRID_MAP_FILTERS__COMBINEDMAPFILTER_HPP_
#define GRID_MAP_FILTERS__COMBINEDMAPFILTER_HPP_
#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <Eigen/Core>
#include <string>
#include <vector>
#include "EigenLab/EigenLab.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <atomic>
#include <thread>


namespace grid_map
{
/*!
 * Computes the Conditional Value-at-Risk (CVaR) of values inside a sliding window
 * on a given input layer, and writes the result to a new output layer.
 */
template<typename T>
class CombinedMapFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor.
   */
  CombinedMapFilter();

  /*!
   * Destructor.
   */
  virtual ~CombinedMapFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Applies the CVaR calculation over a sliding window.
   * @param mapIn Input grid map.
   * @param mapOut Output grid map with added CVaR layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:

  EigenLab::Parser<Eigen::MatrixXf> parser_;
  //! Input layer name.
  std::vector<std::string> inputLayers_;

  //! Output layer name.
  std::string outputLayer_;

  double we_, wr_;

  // clock subscription internals
  rclcpp::Node::SharedPtr clockNode_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub_;
  rclcpp::executors::SingleThreadedExecutor executor_;   
  std::atomic<double> currentSimTime_{0.0};
  std::thread spinThread_;
  std::atomic<bool> running_{false};
};

}  // namespace grid_map

#endif  // GRID_MAP_FILTERS__COMBINEDMAPFILTER_HPP_