from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler,IncludeLaunchDescription
from launch.conditions import IfCondition,UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Whether to run in simulation (Gazebo)",
        )
    )

    # Initialize Arguments
    use_sim = LaunchConfiguration("use_sim")

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("myagv_plus_controller"),
            "config",
            "controllers.yaml",
        ]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("myagv_plus_description"),
                "urdf",
                "myagv_plus.urdf.xacro"
            ]),
            " ",
            "use_sim:=", use_sim,
            " ",
            "controller_config:=", robot_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        condition=IfCondition(use_sim),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("mecanum_drive_controller/reference_unstamped", "/cmd_vel"),
            ("mecanum_drive_controller/odom", "/odom"),   
        ],
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
    )


    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            *declared_arguments,
            control_node,
            gz_sim,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )