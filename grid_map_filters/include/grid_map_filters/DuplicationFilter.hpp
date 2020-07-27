/*
 * DuplicationFilter.hpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__DUPLICATIONFILTER_HPP_
#define GRID_MAP_FILTERS__DUPLICATIONFILTER_HPP_

#include <filters/filter_base.hpp>

#include <string>

namespace grid_map
{

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
  bool configure() override;

  /*!
   * Duplicates the specified layers of a grid map.
   * @param mapIn with the layer to duplicate.
   * @param mapOut with the layer duplicated.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Name of the layer that is duplicated.
  std::string inputLayer_;

  //! Name of the new layer.
  std::string outputLayer_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__DUPLICATIONFILTER_HPP_
