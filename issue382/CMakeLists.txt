cmake_minimum_required(VERSION 3.15)
project(issue382)

find_package(grid_map_core CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)


add_executable(main1 main1.cpp)
target_link_libraries(main1 PRIVATE grid_map_core::grid_map_core)


add_executable(main2 main2.cpp)
target_link_libraries(main2 PRIVATE Eigen3::Eigen)