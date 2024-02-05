cmake_minimum_required(VERSION 3.5)
project(grid_map_core)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)

## Define Eigen addons.
include(cmake/${PROJECT_NAME}-extras.cmake)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

grid_map_package()

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

## Specify additional locations of header files
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
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
  # Linting is setup this way to add a filter
  # to ament_cpplint to ignore the lack of
  # copyright messages at the top of files.
  # Copyright messages are being checked for by both
  # ament_cmake_cpplint & ament_cmake_copyright.

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # exclude copyright checks
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_cpplint
      ament_cmake_copyright
    )
    ament_lint_auto_find_test_dependencies()

    # run cpplint without copyright filter
    find_package(ament_cmake_cpplint)
    ament_cpplint(
      FILTERS -legal/copyright -build/include_order
    )
  endif()
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package(CONFIG_EXTRAS
  cmake/${PROJECT_NAME}-extras.cmake
)
