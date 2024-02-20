ament_add_gtest(${PROJECT_NAME}-test
  test_grid_map_costmap_2d.cpp
  test_costmap_2d_converter.cpp
)

target_link_libraries(${PROJECT_NAME}-test
  ${PROJECT_NAME}::${PROJECT_NAME}
)

ament_add_gtest(costmap-2d-ros-test
  test_costmap_2d_ros.cpp
)

target_link_libraries(costmap-2d-ros-test
  ${PROJECT_NAME}::${PROJECT_NAME}
)

