cmake_minimum_required(VERSION 3.5)
project(grid_map_filters)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)

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
  
grid_map_package()

set(dependencies
  TBB
  filters
  grid_map_core
  grid_map_msgs
  grid_map_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${TBB_INCLUDE_DIRS}
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

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
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
  FILES filter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include 
  ${TBB_INCLUDE_DIRS}
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
endif()

ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()
