ament_add_gtest(${PROJECT_NAME}-test
  test_grid_map_costmap_2d.cpp
  test_costmap_2d_converter.cpp
)

ament_target_dependencies(${PROJECT_NAME}-test
  ${dependencies}
)

ament_add_gtest(costmap-2d-ros-test
  test_costmap_2d_ros.cpp
)

ament_target_dependencies(costmap-2d-ros-test
  ${dependencies}
)
