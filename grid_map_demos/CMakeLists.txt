cmake_minimum_required(VERSION 3.5.1)
project(grid_map_demos)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  cv_bridge
  octomap_msgs
  filters
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(octomap REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(simple_demo
  src/simple_demo_node.cpp
)

add_executable(tutorial_demo
  src/tutorial_demo_node.cpp
)

add_executable(iterators_demo
  src/iterators_demo_node.cpp
  src/IteratorsDemo.cpp
)

add_executable(image_to_gridmap_demo
  src/image_to_gridmap_demo_node.cpp
  src/ImageToGridmapDemo.cpp
)

add_executable(grid_map_to_image_demo
  src/grid_map_to_image_demo_node.cpp
  src/GridmapToImageDemo.cpp
)

add_executable(octomap_to_gridmap_demo
  src/octomap_to_gridmap_demo_node.cpp
  src/OctomapToGridmapDemo.cpp
)

add_executable(move_demo
  src/move_demo_node.cpp
)

add_executable(iterator_benchmark
  src/iterator_benchmark.cpp
)

add_executable(opencv_demo
  src/opencv_demo_node.cpp
)

add_executable(resolution_change_demo
  src/resolution_change_demo_node.cpp
)

add_executable(filters_demo
  src/filters_demo_node.cpp
  src/FiltersDemo.cpp
)

add_executable(normal_filter_comparison_demo
  src/normal_filter_comparison_node.cpp
)

add_executable(interpolation_demo
  src/interpolation_demo_node.cpp
  src/InterpolationDemo.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  simple_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  tutorial_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  iterators_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  image_to_gridmap_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  grid_map_to_image_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  octomap_to_gridmap_demo
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_link_libraries(
  move_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  iterator_benchmark
  ${catkin_LIBRARIES}
)

target_link_libraries(
  opencv_demo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  resolution_change_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  filters_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  normal_filter_comparison_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  interpolation_demo
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS scripts/image_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(
  TARGETS 
    filters_demo
    image_to_gridmap_demo
    grid_map_to_image_demo
    interpolation_demo
    iterator_benchmark
    iterators_demo
    move_demo
    normal_filter_comparison_demo
    octomap_to_gridmap_demo
    opencv_demo
    resolution_change_demo
    simple_demo
    tutorial_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY config data doc launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/empty_test.cpp
  )
  add_dependencies(${PROJECT_NAME}-test
    filters_demo
    image_to_gridmap_demo
    grid_map_to_image_demo
    interpolation_demo
    iterator_benchmark
    iterators_demo
    move_demo
    normal_filter_comparison_demo
    octomap_to_gridmap_demo
    opencv_demo
    resolution_change_demo
    simple_demo
    tutorial_demo
  )
  target_include_directories(${PROJECT_NAME}-test PRIVATE
    include
  )
  target_include_directories(${PROJECT_NAME}-test SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}-test
    gtest_main
    ${catkin_LIBRARIES}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        ${PROJECT_NAME}-test
    )
  endif()
endif()

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)