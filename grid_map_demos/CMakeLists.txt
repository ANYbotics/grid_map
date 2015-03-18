cmake_minimum_required(VERSION 2.8.3)
project(grid_map_demos)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  grid_map
  grid_map_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
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
)

## Declare a cpp library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/grid_map_example_node.cpp
# )

## Declare a cpp executable
add_executable(
	simple_demo
	src/simple_demo_node.cpp
)

add_executable(
	iterators_demo
	src/iterators_demo_node.cpp
	src/IteratorsDemo.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  simple_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(
  iterators_demo
  ${catkin_LIBRARIES}
)
