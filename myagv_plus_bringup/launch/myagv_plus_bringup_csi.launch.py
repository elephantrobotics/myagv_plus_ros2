from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    enable_esp32 = LaunchConfiguration("enable_esp32")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the robot"
    )

    declare_enable_esp32 = DeclareLaunchArgument(
        "enable_esp32",
        default_value="true",
        description="Enable ESP32 driver node"
    )

    declare_esp32_debug_mode = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Enable debug mode for ESP32 driver (logs sent and received frames)"
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("myagv_plus_bringup"),
                "launch",
                "myagv_plus_bringup.launch.py"
            ])
        ),
        launch_arguments={
            "namespace": namespace,
            "enable_esp32": enable_esp32,
            "enable_csi_camera": "true",
            "debug": LaunchConfiguration("debug"),
        }.items()
    )

    return LaunchDescription([
        declare_namespace,
        declare_enable_esp32,
        declare_esp32_debug_mode,
        GroupAction([
            SetRemap(src="voltage", dst="/voltage_hw"),
            SetRemap(src="voltage_backup", dst="/voltage_backup_hw"),
            bringup_launch,
        ]),
    ])
