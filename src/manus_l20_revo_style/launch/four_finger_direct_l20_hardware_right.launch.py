from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("manus_l20_revo_style"), "config", "four_finger_direct_l20_right_hardware.yaml"]
    )
    command_topic = "/cb_right_hand_control_cmd"
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/manus_glove_0"),
            DeclareLaunchArgument("can", default_value="can0"),
            DeclareLaunchArgument("driver_speed", default_value="35,35,35,35,35"),
            DeclareLaunchArgument("driver_command_hz", default_value="30.0"),
            DeclareLaunchArgument("driver_state_hz", default_value="10.0"),
            DeclareLaunchArgument("driver_can_sleep_ms", default_value="3.0"),
            DeclareLaunchArgument("is_touch", default_value="false"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "linker_hand_ros2_sdk",
                    "linker_hand_advanced_g20",
                    "--hand_type",
                    "right",
                    "--can",
                    LaunchConfiguration("can"),
                    "--is_touch",
                    LaunchConfiguration("is_touch"),
                    "--speed",
                    LaunchConfiguration("driver_speed"),
                    "--command_hz",
                    LaunchConfiguration("driver_command_hz"),
                    "--state_hz",
                    LaunchConfiguration("driver_state_hz"),
                    "--can_sleep_ms",
                    LaunchConfiguration("driver_can_sleep_ms"),
                    "--ros-args",
                    "-r",
                    "__node:=linker_hand_advanced_g20_right",
                ],
                output="screen",
            ),
            Node(
                package="manus_l20_revo_style",
                executable="revo_style_l20_node",
                name="manus_l20_four_finger_direct_hardware_right",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "command_topic": command_topic,
                        "config_path": config_path,
                        "publish_rate_hz": 60.0,
                        "log_every_n": 20,
                    }
                ],
            ),
        ]
    )
