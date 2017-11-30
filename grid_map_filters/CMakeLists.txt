cmake_minimum_required(VERSION 2.8.3)
project(grid_map_filters)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_ros
  grid_map_msgs
  filters
)

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)

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
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    grid_map_ros
    grid_map_core
    grid_map_msgs
    filters
  DEPENDS
    #Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/ThresholdFilter.cpp
  src/MinInRadiusFilter.cpp
  src/MeanInRadiusFilter.cpp
  src/NormalVectorsFilter.cpp
  src/CurvatureFilter.cpp
  src/NormalColorMapFilter.cpp
  src/LightIntensityFilter.cpp
  src/MathExpressionFilter.cpp
  src/SlidingWindowMathExpressionFilter.cpp
  src/DuplicationFilter.cpp
  src/DeletionFilter.cpp
  src/ColorFillFilter.cpp
  src/ColorMapFilter.cpp
  src/ColorBlendingFilter.cpp
  src/SetBasicLayersFilter.cpp
  src/BufferNormalizerFilter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  FILES filter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
