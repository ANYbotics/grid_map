##############################################################################
# Unit Tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.
##############################################################################

find_package(rostest REQUIRED)

##############################################################################
# Macro
##############################################################################

macro(${PROJECT_NAME}_add_rostest test_name)
  add_rostest_gtest(test_${test_name} ${test_name}/${test_name}.test ${test_name}/${test_name}.cpp)
  if (TARGET test_${test_name})
    add_dependencies(test_${test_name} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_${test_name} ${catkin_LIBRARIES})
  endif()
endmacro()

##############################################################################
# Tests
##############################################################################

grid_map_costmap_2d_add_rostest(costmap_2d_ros)
