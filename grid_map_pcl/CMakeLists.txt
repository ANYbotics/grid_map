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

find_package(PCL REQUIRED
  COMPONENTS
    common
    features
    filters
    io
    kdtree
    segmentation
    surface
)

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
    PCL
)

###########
## Build ##
###########

# Library.
add_library(${PROJECT_NAME}
  ${SRC_FILES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_FEATURES_INCLUDE_DIRS}
  ${PCL_FILTERS_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${PCL_KDTREE_INCLUDE_DIRS}
  ${PCL_SEGMENTATION_INCLUDE_DIRS}
  ${PCL_SURFACE_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  yaml-cpp
)


# Nodes.
add_executable(grid_map_pcl_loader_node
  src/grid_map_pcl_loader_node.cpp
)
add_dependencies(grid_map_pcl_loader_node
  ${PROJECT_NAME}
)
target_include_directories(grid_map_pcl_loader_node PRIVATE
  include
)
target_include_directories(grid_map_pcl_loader_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(grid_map_pcl_loader_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(pointcloud_publisher_node
  src/pointcloud_publisher_node.cpp
)
add_dependencies(pointcloud_publisher_node
  ${PROJECT_NAME}
)
target_include_directories(pointcloud_publisher_node PRIVATE
  include
)
target_include_directories(pointcloud_publisher_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(pointcloud_publisher_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
    grid_map_pcl_loader_node
    pointcloud_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
install(
  DIRECTORY
    doc
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  DIRECTORY
    test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.pcd"
)
install(
  FILES
    README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
      roslaunch
  )

  roslaunch_add_file_check(launch)

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

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
