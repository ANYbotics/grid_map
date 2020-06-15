cmake_minimum_required(VERSION 3.5)

project(grid_map_cmake_helpers NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "grid_map_cmake_helpers-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
