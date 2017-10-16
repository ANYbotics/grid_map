/*
 * IndexCheckerNonZero.cpp
 *
 *  Created on: Oct 14, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/IndexCheckerNonZero.hpp"

namespace grid_map {

IndexCheckerNonZero::IndexCheckerNonZero(const GridMap& map, const std::string& layer):
    IndexChecker(map),
    layer_(layer){}

bool IndexCheckerNonZero::check(const Index& index) const{
  return (map_.at(layer_, index) != 0.0);
}

}  // namespace grid_map


