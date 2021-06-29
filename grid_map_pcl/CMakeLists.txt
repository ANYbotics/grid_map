cmake_minimum_required(VERSION 3.5)
project(grid_map_pcl)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(PCL REQUIRED) ## See ROS REP2000 for expected PCL version
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

find_package(OpenMP QUIET)
if(OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DGRID_MAP_PCL_OPENMP_FOUND=${OpenMP_FOUND})
endif()

# Add links to PCL definitons and directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

grid_map_package()

set(SRC_FILES
  src/GridMapPclLoader.cpp
  src/PclLoaderParameters.cpp
  src/PointcloudProcessor.cpp
  src/helpers.cpp
  src/GridMapPclConverter.cpp
)

set(dependencies
  grid_map_core
  grid_map_msgs
  grid_map_ros
  rclcpp
  rcutils
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenMP_CXX_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(grid_map_pcl_loader_node
  src/grid_map_pcl_loader_node.cpp
)

target_link_libraries(grid_map_pcl_loader_node
  ${PROJECT_NAME}
  ${OpenMP_CXX_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    grid_map_pcl_loader_node
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

install(
  DIRECTORY
    config
    launch
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
      FILTERS -legal/copyright
    )
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Adding test data to the package share directory
  install(
    DIRECTORY test/test_data/
    DESTINATION share/${PROJECT_NAME}/test_data/
  )

  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(${PROJECT_NAME}-test
    test/GridMapPclLoaderTest.cpp
    test/HelpersTest.cpp
    test/PointcloudCreator.cpp
    test/PointcloudProcessorTest.cpp
    test/test_helpers.cpp
    test/test_grid_map_pcl.cpp
    TIMEOUT 300
  )
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
