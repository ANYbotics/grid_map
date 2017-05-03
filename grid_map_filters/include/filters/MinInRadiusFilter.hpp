/*
 * MinInRadiusFilter.hpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef MININRADIUSFILTER_HPP
#define MININRADIUSFILTER_HPP

#include <filters/filter_base.h>

#include <vector>
#include <string>

namespace grid_map_filters {

/*!
 * Step Filter class to the minimal value inside a radius.
 */
template<typename T>
class MinInRadiusFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  MinInRadiusFilter();

  /*!
   * Destructor.
   */
  virtual ~MinInRadiusFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes the step traversability value based on an elevation map and
   * saves it as additional grid map layer.
   * The step traversability is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means not traversable. NAN indicates unknown values (terrain).
   * @param mapIn grid map containing elevation map and surface normals.
   * @param mapOut grid map containing mapIn and step traversability values.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Maximum allowed step.
  double radius_;

  //! input layer name
  std::string inputLayer_;
  //! Step map type.
  std::string type_;
};

} /* namespace */

#endif
