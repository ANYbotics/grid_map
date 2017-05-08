/*
 * InpaintFilter.hpp
 *
 *  Created on: May 6, 2017
 *      Author: Tanja Baumann
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef INPAINTFILTER_HPP
#define INPAINTFILTER_HPP

#include <filters/filter_base.h>

//For inpainting
#include "grid_map_cv/grid_map_cv.hpp"
#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

namespace grid_map_filters {

/*!
 * Inpaint Filter class to use to OpenCV function inpaint to fill in holes in the input layer
 */
template<typename T>
class InpaintFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
	InpaintFilter();

  /*!
   * Destructor.
   */
  virtual ~InpaintFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Adds a new output layer to the map
   * Uses the OpenCV function inpaint to fill in holes in the input layer
   * Saves to filled map in the outputlayer
   * @param mapIn grid map containing input layer
   * @param mapOut grid map containing mapIn and inpainted input layer.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! inapainting radius
  double radius_;
  //! input layer name
  std::string inputLayer_;
  //! Step map type.
  std::string type_;
};

} /* namespace */

#endif
