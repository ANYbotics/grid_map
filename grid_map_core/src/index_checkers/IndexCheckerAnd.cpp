/*
 * IndexCheckerAnd.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Perry Franklin
 */


#include "grid_map_core/index_checkers/IndexCheckerAnd.hpp"

namespace grid_map {

IndexCheckerAnd::IndexCheckerAnd(const GridMap& map):
    IndexChecker(map){}

IndexCheckerAnd::~IndexCheckerAnd(){
}

bool IndexCheckerAnd::check(const Index& index) const{
  for (const std::unique_ptr<IndexChecker>& cur_checker: checks_){
    if (!cur_checker->check(index)){
      return false;
    }
  }
  return true;
}

void IndexCheckerAnd::addChecker(const IndexChecker& checker){
  checks_.emplace_back(checker.clone());
}

std::unique_ptr<IndexChecker> IndexCheckerAnd::clone() const{
  std::unique_ptr<IndexCheckerAnd> to_return(new IndexCheckerAnd(map_));

  for (const std::unique_ptr<IndexChecker>& element: checks_){
    to_return->addChecker(*element);
  }

  return to_return;
}

}  // namespace grid_map





