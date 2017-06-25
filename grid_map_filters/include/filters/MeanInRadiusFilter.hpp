/*
 * MeanInRadiusFilter.hpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef MEANINRADIUSFILTER_HPP
#define MEANINRADIUSFILTER_HPP

#include <filters/filter_base.h>

#include <vector>
#include <string>

namespace grid_map_filters {

/*!
 * Filter class to find the mean of the values inside a radius.
 */
template<typename T>
class MeanInRadiusFilter : public filters::FilterBase<T> {

 public:
  /*!
   * Constructor
   */
  MeanInRadiusFilter();

  /*!
   * Destructor.
   */
  virtual ~MeanInRadiusFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes for each value in the input layer the mean of all values in a radius around it
   * Saves this mean in an additional output layer
   * @param mapIn grid map containing the input layer.
   * @param mapOut grid map containing mapIn and smoothed input layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! radius to take the mean in
  double radius_;

  //! input layer name
  std::string inputLayer_;
  //! map type.
  std::string type_;
};

} /* namespace */

#endif
