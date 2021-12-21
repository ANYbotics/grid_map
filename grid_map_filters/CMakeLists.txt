cmake_minimum_required(VERSION 3.5.1)
project(grid_map_filters)

# Better with serial algorithms.
set(CMAKE_CXX_STANDARD 11)
# We want performance (fast-math) but also need a representation for NaN values to represent missing values. 
# Therefore, we disable the finite-math-only flag that was set by fast-math.  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -fno-finite-math-only")
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Other possible options.
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native -ffast-math")
#set(TARGET_ARCHITECTURE "kaby-lake")

## Create a list of catkin package dependencies used in header files of this package.
set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  grid_map_core
  grid_map_ros
  grid_map_msgs
  filters
)

## Create a list of catkin package dependencies, now for both header and source files.
set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

## Find catkin dependencies for building this package.
find_package(catkin REQUIRED
  COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

pkg_check_modules(TBB "tbb")
if (TBB_FOUND)
  add_definitions(
    ${TBB_CFLAGS}
    ${TBB_CFLAGS_OTHER}
  )
else()
  message([FATAL_ERROR] "tbb module not found")
endif ()

find_package(OpenCV REQUIRED)

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
    ${OpenCV_INCLUDE_DIRS} # TODO Remove this include directory, currently it exported by the medianFillFilter header.
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
  DEPENDS
    OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/ThresholdFilter.cpp
  src/MinInRadiusFilter.cpp
  src/MeanInRadiusFilter.cpp
  src/MedianFillFilter.cpp
  src/MockFilter.cpp
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

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${TBB_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TBB_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Instantiate plugin exports for every filter in this package.
add_library(${PROJECT_NAME}_plugins
  src/plugins.cpp
)

target_include_directories(${PROJECT_NAME}_plugins
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
  ${TBB_LIBRARIES} # is this necessary?
  ${OpenCV_LIBRARIES} # is this necessary?
)

add_dependencies(${PROJECT_NAME}_plugins
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  FILES
    filter_plugins.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_grid_map_filters.cpp
    test/median_fill_filter_test.cpp
    test/mock_filter_test.cpp
    test/threshold_filter_test.cpp
  )
  target_include_directories(${PROJECT_NAME}-test PRIVATE
    include
  )
  target_include_directories(${PROJECT_NAME}-test SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}-test
    gmock
    gtest
    ${PROJECT_NAME}
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