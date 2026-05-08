import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions


def generate_launch_description():

    rviz_config_dir = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'rviz',
        'gmapping.rviz')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping', executable='slam_gmapping', output='screen'),
    
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

    ])
