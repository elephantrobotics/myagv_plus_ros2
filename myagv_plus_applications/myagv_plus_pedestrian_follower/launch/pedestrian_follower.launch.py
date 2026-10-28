from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    namespace = LaunchConfiguration("namespace")

    declared_arguments = [
        DeclareLaunchArgument("use_sim", default_value="false", description="Simulation mode"),
        DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    ]

    # 控制器启动
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("myagv_plus_controller"), "launch", "controller.launch.py"])
        ),
        launch_arguments={"use_sim": use_sim, "namespace": namespace}.items(),
    )

    # 手势控制摄像头节点
    camera_node = Node(
        package="myagv_plus_pedestrian_follower",
        executable="gesture_control",
        output="screen",
        parameters=[
            {"sensor_id": 0},
            {"capture_width": 1280},
            {"capture_height": 720},
            {"framerate": 30},
            {"flip_method": 2},
            {"frame_id": "camera_link"},
        ]
    )

    # 行人跟随节点
    pedestrian_follower_node = Node(
        package="myagv_plus_pedestrian_follower",
        executable="pedestrian_follower",
        output="screen",
        parameters=[
            {"image_topic": "/camera/image_raw"},
            {"cmd_vel_topic": "/cmd_vel"},
            {"min_distance": 0.5},
            {"max_distance": 5.0},
            {"max_linear_speed": 0.25},
            {"max_angular_speed": 1.0}
        ]
    )

    return LaunchDescription([*declared_arguments, controller_launch, camera_node, pedestrian_follower_node])