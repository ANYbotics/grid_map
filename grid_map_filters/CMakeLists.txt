cmake_minimum_required(VERSION 3.5)
project(grid_map_filters)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pluginlib REQUIRED)

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
  pluginlib
)

set(filter_libs
  threshold_filter
  min_in_radius_filter
  mean_in_radius_filter
  normal_vectors_filter
  curvature_filter
  normal_color_map_filter
  light_intensity_filter
  math_expression_filter
  sliding_window_math_expression_filter
  duplication_filter
  deletion_filter
  color_fill_filter
  color_map_filter
  color_blending_filter
  set_basic_layers_filter
  buffer_normalizer_filter
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

## Declare cpp libraries
add_library(threshold_filter SHARED src/ThresholdFilter.cpp)
add_library(min_in_radius_filter SHARED src/MinInRadiusFilter.cpp)
add_library(mean_in_radius_filter SHARED src/MeanInRadiusFilter.cpp)
add_library(normal_vectors_filter SHARED src/NormalVectorsFilter.cpp)
add_library(curvature_filter SHARED src/CurvatureFilter.cpp)
add_library(normal_color_map_filter SHARED src/NormalColorMapFilter.cpp)
add_library(light_intensity_filter SHARED src/LightIntensityFilter.cpp)
add_library(math_expression_filter SHARED src/MathExpressionFilter.cpp)
add_library(sliding_window_math_expression_filter SHARED src/SlidingWindowMathExpressionFilter.cpp)
add_library(duplication_filter SHARED src/DuplicationFilter.cpp)
add_library(deletion_filter SHARED src/DeletionFilter.cpp)
add_library(color_fill_filter SHARED src/ColorFillFilter.cpp)
add_library(color_map_filter SHARED src/ColorMapFilter.cpp)
add_library(color_blending_filter SHARED src/ColorBlendingFilter.cpp)
add_library(set_basic_layers_filter SHARED src/SetBasicLayersFilter.cpp)
add_library(buffer_normalizer_filter SHARED src/BufferNormalizerFilter.cpp)

foreach(lib_name ${filter_libs})
  ament_target_dependencies(${lib_name}
    ${dependencies}
  )

  target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-expand=1")
  target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
endforeach()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${filter_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
ament_export_libraries(${filter_libs})
pluginlib_export_plugin_description_file(filters filter_plugins.xml)
ament_package()
