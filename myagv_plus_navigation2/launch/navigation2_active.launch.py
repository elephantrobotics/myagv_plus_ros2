import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('myagv_plus_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = 'myagvplus.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('myagv_plus_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('myagv_plus_navigation2'),
        'rviz',
        'myagv_plus_navigation2.rviz')

    declare_map_arg = DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'
    )

    declare_params_file_arg = DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'
    )

    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'params_file': param_dir}.items(),
    )

    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
    )

    return LaunchDescription(
        [
            declare_map_arg,
            declare_params_file_arg,
            nav2_launch,
            rviz2_node
        ]
    )