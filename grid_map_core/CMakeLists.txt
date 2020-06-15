cmake_minimum_required(VERSION 3.5)
project(grid_map_core)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)

## Define Eigen addons. 
include(cmake/${PROJECT_NAME}-extras.cmake)

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
# Solution to find Eigen3 with Saucy.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

grid_map_package()

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/GridMap.cpp
   src/GridMapMath.cpp
   src/SubmapGeometry.cpp
   src/BufferRegion.cpp
   src/Polygon.cpp
   src/CubicInterpolation.cpp
   src/iterators/GridMapIterator.cpp
   src/iterators/SubmapIterator.cpp
   src/iterators/CircleIterator.cpp
   src/iterators/EllipseIterator.cpp
   src/iterators/SpiralIterator.cpp
   src/iterators/PolygonIterator.cpp
   src/iterators/LineIterator.cpp
   src/iterators/SlidingWindowIterator.cpp
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  
  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_grid_map_core.cpp
    test/test_helpers.cpp
    test/CubicConvolutionInterpolationTest.cpp
    test/CubicInterpolationTest.cpp
    test/GridMapMathTest.cpp
    test/GridMapTest.cpp
    test/GridMapIteratorTest.cpp
    test/LineIteratorTest.cpp
    test/EllipseIteratorTest.cpp
    test/SubmapIteratorTest.cpp
    test/PolygonIteratorTest.cpp
    test/PolygonTest.cpp
    test/EigenPluginsTest.cpp
    test/SpiralIteratorTest.cpp
    test/SlidingWindowIteratorTest.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
