cmake_minimum_required(VERSION 3.5)
project(grid_map_loader)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)

grid_map_package()

set(dependencies
  grid_map_msgs
  grid_map_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

# Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/grid_map_loader_node.cpp
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/GridMapLoader.cpp
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

#############
## Install ##
#############

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()
