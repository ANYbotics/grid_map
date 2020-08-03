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
if(TBB_FOUND)
add_definitions(
  ${TBB_CFLAGS}
  ${TBB_CFLAGS_OTHER}
)
else()
message([FATAL_ERROR] "tbb module not found")
endif()

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
add_library(ThresholdFilter src/ThresholdFilter.cpp)
list(APPEND filter_libs ThresholdFilter)

add_library(MinInRadiusFilter src/MinInRadiusFilter.cpp)
list(APPEND filter_libs MinInRadiusFilter)

add_library(MeanInRadiusFilter src/MeanInRadiusFilter.cpp)
list(APPEND filter_libs MeanInRadiusFilter)

add_library(NormalVectorsFilter src/NormalVectorsFilter.cpp)
list(APPEND filter_libs NormalVectorsFilter)

add_library(CurvatureFilter src/CurvatureFilter.cpp)
list(APPEND filter_libs CurvatureFilter)

add_library(NormalColorMapFilter src/NormalColorMapFilter.cpp)
list(APPEND filter_libs NormalColorMapFilter)

add_library(LightIntensityFilter src/LightIntensityFilter.cpp)
list(APPEND filter_libs LightIntensityFilter)

add_library(MathExpressionFilter src/MathExpressionFilter.cpp)
list(APPEND filter_libs MathExpressionFilter)

add_library(SlidingWindowMathExpressionFilter src/SlidingWindowMathExpressionFilter.cpp)
list(APPEND filter_libs SlidingWindowMathExpressionFilter)

add_library(DuplicationFilter src/DuplicationFilter.cpp)
list(APPEND filter_libs DuplicationFilter)

add_library(DeletionFilter src/DeletionFilter.cpp)
list(APPEND filter_libs DeletionFilter)

add_library(ColorFillFilter src/ColorFillFilter.cpp)
list(APPEND filter_libs ColorFillFilter)

add_library(ColorMapFilter src/ColorMapFilter.cpp)
list(APPEND filter_libs ColorMapFilter)

add_library(ColorBlendingFilter src/ColorBlendingFilter.cpp)
list(APPEND filter_libs ColorBlendingFilter)

add_library(SetBasicLayersFilter src/SetBasicLayersFilter.cpp)
list(APPEND filter_libs SetBasicLayersFilter)

add_library(BufferNormalizerFilter src/BufferNormalizerFilter.cpp)
list(APPEND filter_libs BufferNormalizerFilter)

foreach(filter_lib ${filter_libs})
  ament_target_dependencies(${filter_lib} ${dependencies})
endforeach()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${filter_libs}
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

ament_export_include_directories(
  include
  ${TBB_INCLUDE_DIRS}
)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()
