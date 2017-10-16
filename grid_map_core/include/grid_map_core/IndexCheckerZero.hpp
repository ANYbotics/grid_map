/*
 * IndexCheckerZero.hpp
 *
 *  Created on: Oct 16, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include "grid_map_core/IndexChecker.hpp"

namespace grid_map {

class IndexCheckerZero : public IndexChecker {

public:

  IndexCheckerZero(const GridMap& map, const std::string& layer);

  bool check(const Index& index) const override;

private:

  const std::string layer_;

};

}  // namespace grid_map
