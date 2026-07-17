from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("manus_l20_revo_style"), "config", "four_finger_direct_l20_right.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/manus_glove_0"),
            DeclareLaunchArgument("command_topic", default_value="~/l20_command"),
            Node(
                package="manus_l20_revo_style",
                executable="revo_style_l20_node",
                name="manus_l20_four_finger_direct_right",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "config_path": config_path,
                    }
                ],
            ),
        ]
    )
