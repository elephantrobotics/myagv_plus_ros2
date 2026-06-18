from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


RED = '\033[31m'
RESET = '\033[0m'


def on_exit_log(name):
    def callback(event, context):
        if event.returncode == 0:
            return []
        return [LogInfo(msg=f'{RED}[smart_logistics_kit] {name} exited with code {event.returncode}.{RESET}')]
    return callback


def status_handlers(node, name):
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=on_exit_log(name)
            )
        ),
    ]


def generate_launch_description():
    aruco_tracker_node = Node(
        package='myagv_plus_aruco_tracker',
        executable='aruco_tracker',
        name='aruco_tracker',
        output='screen',
        emulate_tty=True,
    )

    parking_node = Node(
        package='smart_logistics_kit',
        executable='parking',
        name='parking',
        output='screen',
        emulate_tty=True,
    )

    autocharge_coordinator_node = Node(
        package='smart_logistics_kit',
        executable='autocharge_coordinator',
        name='autocharge_coordinator',
        output='screen',
        emulate_tty=True,
    )

    mission_node = Node(
        package='smart_logistics_kit',
        executable='main',
        name='logistics_route_mission',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        *status_handlers(aruco_tracker_node, 'aruco_tracker'),
        *status_handlers(parking_node, 'parking'),
        *status_handlers(autocharge_coordinator_node, 'autocharge_coordinator'),
        *status_handlers(mission_node, 'logistics_route_mission'),
        aruco_tracker_node,
        parking_node,
        autocharge_coordinator_node,
        mission_node,
    ])
