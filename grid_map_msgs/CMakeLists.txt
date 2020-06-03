cmake_minimum_required(VERSION 3.5)
project(grid_map_msgs)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_generation REQUIRED)

grid_map_package()

#######################################
## Declare ROS messages and services ##
#######################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GridMapInfo.msg"
  "msg/GridMap.msg"
  "srv/SetGridMap.srv"
  "srv/GetGridMap.srv"
  "srv/GetGridMapInfo.srv"
  "srv/ProcessFile.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

