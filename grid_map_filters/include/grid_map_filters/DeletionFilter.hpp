/*
 * DeletionFilter.hpp
 *
 *  Created on: Mar 19, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_FILTERS__DELETIONFILTER_HPP_
#define GRID_MAP_FILTERS__DELETIONFILTER_HPP_

#include <filters/filter_base.hpp>

#include <vector>
#include <string>

namespace grid_map
{

/*!
 * Deletion filter class deletes layers of a grid map.
 */
template<typename T>
class DeletionFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  DeletionFilter();

  /*!
   * Destructor.
   */
  virtual ~DeletionFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Deletes the specified layers of a grid map.
   * @param mapIn gridMap with the different layers.
   * @param mapOut gridMap without the deleted layers.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! List of layers that should be deleted.
  std::vector<std::string> layers_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__DELETIONFILTER_HPP_
