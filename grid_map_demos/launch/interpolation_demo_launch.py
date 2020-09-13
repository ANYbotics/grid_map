import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')

    config_file = LaunchConfiguration('param_file')
    rviz_config_file = LaunchConfiguration('rviz_config')

    config_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'interpolation_demo.yaml'),
        description='Full path to the parameter config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'rviz', 'interpolation_demo.rviz'),
        description='Full path to the RVIZ config file to use')

    interpolation_demo_node = Node(
        package='grid_map_demos',
        executable='interpolation_demo',
        name='grid_map_interpolation_demo',
        output='screen',
        parameters=[config_file]
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

    ld.add_action(config_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(interpolation_demo_node)
    ld.add_action(rviz2_node)

    return ld
