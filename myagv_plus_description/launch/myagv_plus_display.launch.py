import os
from launch import LaunchDescription
from launch_ros.actions import Node,PushRosNamespace
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command,LaunchConfiguration,PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    rviz_config_dir = os.path.join(
        get_package_share_directory('myagv_plus_description'),
        'rviz',
        'myagv_plus_display.rviz')

    urdf_file = os.path.join(
        get_package_share_directory('myagv_plus_description'),
        'urdf',
        'myagv_plus.urdf.xacro'
    )

    robot_description_content = Command([
        'xacro ',
        urdf_file,
    ])

    return LaunchDescription([

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            condition=IfCondition(use_rviz),
            output='screen')

    ])
