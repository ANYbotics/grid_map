/*
 * ZeroOneFilter.hpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef ZEROONEFILTER_HPP
#define ZEROONEFILTER_HPP

#include <filters/filter_base.h>

#include <vector>
#include <string>

namespace grid_map_filters {

/*!
 * Zero One Filter class to set all the values in a layer to 1 if the are bigger than the threshold and to 0 if they are smaller
 */
template<typename T>
class ZeroOneFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
	ZeroOneFilter();

  /*!
   * Destructor.
   */
  virtual ~ZeroOneFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Compares to values in the input layer with the threshold
   * Adds a new output layer to the map
   * Sets the value in the output layer to 1.0 if the corresponding input layer value is larger than the threshold
   * Sets the value in the output layer to 0.0 if the corresponding input layer value is smaller than the threshold
   * @param mapIn grid map containing elevation map and surface normals.
   * @param mapOut grid map containing mapIn and thresholded input layer values.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Maximum allowed step.
  double threshold_;

  //! input layer name
  std::string inputLayer_;
  //! Step map type.
  std::string type_;
};

} /* namespace */

#endif
