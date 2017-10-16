/*
 * IndexCheckerZero.cpp
 *
 *  Created on: Oct 16, 2017
 *      Author: Perry Franklin
 */

#include "grid_map_core/IndexCheckerZero.hpp"
#include <iostream>
namespace grid_map {

IndexCheckerZero::IndexCheckerZero(const GridMap& map, const std::string& layer):
    IndexChecker(map),
    layer_(layer){}

bool IndexCheckerZero::check(const Index& index) const{
  return (map_.at(layer_, index) == 0.0);
}

}  // namespace grid_map


