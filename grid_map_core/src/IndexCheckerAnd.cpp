/*
 * IndexCheckerAnd.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: Perry Franklin
 */


#include "grid_map_core/IndexCheckerAnd.hpp"

namespace grid_map {

IndexCheckerAnd::IndexCheckerAnd(const GridMap& map):
    IndexChecker(map){}

IndexCheckerAnd::~IndexCheckerAnd(){
  for (IndexChecker* element: checks_){
    delete element;
  }
}

bool IndexCheckerAnd::check(const Index& index) const{
  for (IndexChecker* cur_checker: checks_){
    if (!cur_checker->check(index)){
      return false;
    }
  }
  return true;
}

void IndexCheckerAnd::addChecker(const IndexChecker& checker){
  checks_.push_back(checker.clone());
}

IndexChecker* IndexCheckerAnd::clone() const{
  IndexCheckerAnd* to_return = new IndexCheckerAnd(map_);

  for (IndexChecker* element: checks_){
    to_return->addChecker(*element);
  }

  return to_return;
}

}  // namespace grid_map





