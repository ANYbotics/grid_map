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
    filters_config_file = LaunchConfiguration('filters_config')
    visualization_config_file = LaunchConfiguration('visualization_config')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare launch arguments
    declare_filters_config_file_cmd = DeclareLaunchArgument(
        'filters_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'filters_demo_filter_chain.yaml'),
        description='Full path to the filter chain config file to use')

    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'filters_demo.yaml'),
        description='Full path to the Gridmap visualization config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'rviz', 'filters_demo.rviz'),
        description='Full path to the RVIZ config file to use')

    # Declare node actions
    grid_map_filter_demo_node = Node(
        package='grid_map_demos',
        executable='filters_demo',
        name='grid_map_filters',
        output='screen',
        parameters=[filters_config_file]
    )

    image_publisher_node = Node(
        package='grid_map_demos',
        executable='image_publisher.py',
        name='image_publisher',
        output='screen',
        parameters=[{
            'image_path': os.path.join(grid_map_demos_dir, 'data', 'terrain.png'),
            'topic': 'image'
        }]
    )

    image_to_gridmap_demo_node = Node(
        package='grid_map_demos',
        executable='image_to_gridmap_demo',
        name='image_to_gridmap',
        output='screen',
        parameters=[visualization_config_file]
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
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
    ld.add_action(declare_filters_config_file_cmd)
    ld.add_action(declare_visualization_config_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add node actions to the launch description
    ld.add_action(grid_map_filter_demo_node)
    ld.add_action(image_publisher_node)
    ld.add_action(image_to_gridmap_demo_node)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(rviz2_node)

    return ld
