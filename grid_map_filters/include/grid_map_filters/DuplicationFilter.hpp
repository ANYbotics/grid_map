/*
 * DuplicationFilter.hpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
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
class DuplicationFilter : public filters::FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  DuplicationFilter();

  /*!
   * Destructor.
   */
  virtual ~DuplicationFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  virtual bool configure();

  /*!
   * Duplicates the specified layers of a grid map.
   * @param mapIn with the layer to duplicate.
   * @param mapOut with the layer duplicated.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  //! Name of the layer that is duplicated.
  std::string inputLayer_;

  //! Name of the new layer.
  std::string outputLayer_;
};

} /* namespace */
