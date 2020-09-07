cmake_minimum_required(VERSION 3.5.1)
project(grid_map_pcl)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_FILES
  src/GridMapPclConverter.cpp
  src/GridMapPclLoader.cpp
  src/helpers.cpp
  src/PclLoaderParameters.cpp
  src/PointcloudProcessor.cpp
)

set(CATKIN_PACKAGE_DEPENDENCIES  
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pcl_ros
  roscpp
)

find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DGRID_MAP_PCL_OPENMP_FOUND=${OpenMP_FOUND})
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

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
    yaml-cpp
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  yaml-cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(grid_map_pcl_loader_node
  src/grid_map_pcl_loader_node.cpp
)

target_link_libraries(grid_map_pcl_loader_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    grid_map_pcl_loader_node
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

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_grid_map_pcl.cpp
    test/GridMapPclLoaderTest.cpp
    test/HelpersTest.cpp
    test/PointcloudProcessorTest.cpp
    test/test_helpers.cpp
    test/PointcloudCreator.cpp
  )
  target_include_directories(${PROJECT_NAME}-test PRIVATE
    include
  )
  target_include_directories(${PROJECT_NAME}-test SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenMP_CXX_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
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