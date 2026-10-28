import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = 'smart_logistics_kit_lite'


def generate_launch_description():
    pkg_dir = get_package_share_directory(PACKAGE_NAME)
    workspace_dir = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    map_pkg_dir = os.path.join(workspace_dir, 'src', 'myagv_plus_applications', PACKAGE_NAME)
    route_rviz_config = os.path.join(pkg_dir, 'rviz', 'route_tool.rviz')
    default_map = os.path.join(map_pkg_dir, 'map', 'map.yaml')
    map_file = LaunchConfiguration('yaml_filename')

    declare_map_file_cmd = DeclareLaunchArgument(
            'yaml_filename',
            default_value=default_map,
            description='Full path to an occupancy grid map yaml file'
    )

    route_tool_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='route_tool_rviz2',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', route_rviz_config]
    )

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_route_tool',
            output='screen',
            parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']},
            ]
    )

    return LaunchDescription([
            declare_map_file_cmd,
            route_tool_rviz,
            map_server,
            lifecycle_manager
    ])
