cmake_minimum_required(VERSION 3.5)
project(grid_map_octomap)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(OCTOMAP REQUIRED)

message(STATUS "Found Octomap (version ${octomap_VERSION}): ${OCTOMAP_INCLUDE_DIRS}")

###########
## Build ##
###########
include_directories(include)

set( dependencies
  grid_map_core
  OCTOMAP
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/GridMapOctomapConverter.cpp
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

  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_grid_map_octomap.cpp
    test/OctomapConverterTest.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${OCTOMAP_LIBRARIES}
  )
endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include ${OCTOMAP_INCLUDE_DIRS})
ament_export_libraries(
  ${PROJECT_NAME}
  ${OCTOMAP_LIBRARIES}
)
ament_package()
