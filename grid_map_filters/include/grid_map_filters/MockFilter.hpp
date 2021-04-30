/*
 * MockFilter.hpp
 *
 *  Created on: Sep 24, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_base.h>

#include <string>

namespace grid_map {

/*!
 * Duplication filter class duplicates a layer of a grid map.
 */
template<typename T>
class MockFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  MockFilter();

  /*!
   * Destructor.
   */
  virtual ~MockFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Copies the input to the output. The time for the update is specified by processingTime_. Optionally the update is logged.
   * @param mapIn Input.
   * @param mapOut Output.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Flag indicating wheter to also log on update.
  bool printName_;

  //! The time [ms] that the update function takes.
  uint processingTime_;
};

} /* namespace */
