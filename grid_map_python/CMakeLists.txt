cmake_minimum_required(VERSION 3.5)
project(grid_map_python)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-O2 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS pybind11_catkin grid_map_core grid_map_msgs)
find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_python_setup()

pybind11_add_module(grid_map_python SHARED py_module.cpp py_core.cpp)
target_link_libraries(grid_map_python PRIVATE ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(grid_map_python ${grid_map_python_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(grid_map_python
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}
)

install(TARGETS grid_map_python
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
    ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
)
