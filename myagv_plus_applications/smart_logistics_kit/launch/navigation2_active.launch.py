import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


PACKAGE_NAME = 'smart_logistics_kit'


def generate_launch_description():
    pkg_dir = get_package_share_directory(PACKAGE_NAME)
    workspace_dir = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    map_pkg_dir = os.path.join(workspace_dir, 'src', 'myagv_plus_applications', PACKAGE_NAME)
    
    map_dir = os.path.join(map_pkg_dir, 'map', 'map.yaml')
    graph_file = os.path.join(map_pkg_dir, 'graphs', 'map.geojson')
    param_dir = os.path.join(pkg_dir, 'param', 'myagvplus.yaml')
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'smart_logistics_kit_lite.rviz')
    route_rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'route_tool.rviz')
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_mode = LaunchConfiguration('rviz')
    graph_launch_value = PythonExpression(["'' if '", rviz_mode, "' == 'route' else '", graph_file, "'"])

    declare_rviz_cmd = DeclareLaunchArgument(
            'rviz',
            default_value='nav',
            description='RViz mode: nav loads the original navigation RViz, route loads the official Route Tool RViz'
    )
    
    # 1. AMCL 定位（正常）
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_dir,
            'params_file': param_dir,
            'use_sim_time': 'false',
        }.items(),
    )

    navigation_launch = GroupAction(
        actions=[
            SetRemap(src='/goal_pose', dst='/goal_pose_nav2'),
        ] + [
            SetRemap(
                src=f'/{action}/_action/{suffix}',
                dst=f'/{action}_nav2/_action/{suffix}')
            for action in ('navigate_to_pose', 'navigate_through_poses')
            for suffix in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')
        ] + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'params_file': param_dir,
                    'graph': graph_launch_value,
                    'use_sim_time': 'false',
                    'enable_docking': 'false' 
                }.items(),
            ),
        ],
        scoped=True,
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'route'"]))
    )

    route_tool_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': param_dir,
            'graph': graph_launch_value,
            'use_sim_time': 'false',
            'enable_docking': 'false'
        }.items(),
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' == 'route'"]))
    )

    navigate_refiner_proxy_node = Node(
            package='myagv_plus_calibration',
            executable='navigate_to_pose_refiner_proxy',
            name='navigate_to_pose_refiner_proxy',
            output='screen',
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'route'"]))
    )

    final_pose_refiner_node = Node(
            package='myagv_plus_calibration',
            executable='final_pose_refiner',
            name='final_pose_refiner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'final_pose_refiner_auto_start_on_nav_success': False,
            }],
            condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'route'"]))
    )

    nav_rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': False}],
            output='screen',
            condition=IfCondition(PythonExpression(["'", rviz_mode, "' == 'nav'"]))
    )

    route_rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='route_tool_rviz2',
            arguments=['-d', route_rviz_config_dir],
            parameters=[{'use_sim_time': False}],
            output='screen',
            condition=IfCondition(PythonExpression(["'", rviz_mode, "' == 'route'"]))
    )

    route_bridge_node = Node(
            package=PACKAGE_NAME,
            executable='route_bridge',
            name='route_bridge',
            output='screen',
            parameters=[{
                'roadnet_file': graph_file,
                'route_frame': 'map',
                'base_frame': 'base_footprint',
            }],
            condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'route'"]))
    )

    initial_pose_pub = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'],
            output='screen'
    )

    return LaunchDescription([
            declare_rviz_cmd,
            localization_launch,
            navigation_launch,
            route_tool_navigation_launch,
            navigate_refiner_proxy_node,
            final_pose_refiner_node,
            route_bridge_node,
            initial_pose_pub,
            nav_rviz2_node,
            route_rviz2_node
        ])
