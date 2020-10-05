import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the grid_map_demos package share directory
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')

    # Declare launch configuration variables that can access the launch arguments values
    param_file = LaunchConfiguration('param_file')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare launch arguments
    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'octomap_to_gridmap_demo.yaml'),
        description='Full path to the config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'rviz', 'octomap_to_gridmap_demo.rviz'),
        description='Full path to the RVIZ config file to use')

    # Declare node actions

    # TODO(marwan99): re-enable once octomap_mapping is ported to ROS2
    # https://github.com/OctoMap/octomap_mapping/issues/76
    # octomap_server_node = Node(
    #     package='octomap_server',
    #     executable='octomap_server_static',
    #     name='octomap_server',
    #     output='screen',
    #     parameters=[param_file],
    #     arguments=['-d', os.path.join(grid_map_demos_dir + 'data' + 'freiburg1_360.bt']
    # )

    octomap_to_gridmap_demo_node = Node(
        package='grid_map_demos',
        executable='octomap_to_gridmap_demo',
        name='octomap_to_gridmap_demo',
        output='screen'
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[param_file]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments to the launch description
    ld.add_action(declare_param_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add node actions to the launch description
    # ld.add_action(octomap_server_node)
    ld.add_action(octomap_to_gridmap_demo_node)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(rviz2_node)

    return ld
