#include <gtest/gtest.h>

#include "grid_map_filters/MockFilter.hpp"

using namespace grid_map;
using namespace ::testing;

TEST(MockFilter, ConstructFilterTest) {  // NOLINT
  MockFilter mockFilter{};
  SUCCEED();
}