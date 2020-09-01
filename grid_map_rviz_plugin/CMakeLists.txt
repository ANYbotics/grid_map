cmake_minimum_required(VERSION 3.5)
project(grid_map_rviz_plugin)

find_package(ament_cmake REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED COMPONENTS Core Widgets)

grid_map_package()

## Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) # This setting causes Qt's "MOC" generation to happen automatically.

set(dependencies
  grid_map_msgs
  grid_map_ros
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
)

include_directories(
  include
)

set(INCLUDE_FILES_QT
  include/grid_map_rviz_plugin/GridMapDisplay.hpp
)

qt5_wrap_cpp(MOC_FILES ${INCLUDE_FILES_QT})

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

## The list of source files.
## The generated MOC files are included automatically as headers.
set(SOURCE_FILES
  src/GridMapDisplay.cpp
  src/GridMapVisual.cpp
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ${PROJECT_NAME} and specify the list of
## source files we collected above in ${SOURCE_FILES}.
add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
  ${MOC_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

# Install rules
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}/icons
)

install(
  DIRECTORY doc
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
endif()


ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  map_msgs
  nav_msgs
  rclcpp
)

ament_package()
