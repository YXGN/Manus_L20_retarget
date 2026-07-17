from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/manus_glove_0"),
            DeclareLaunchArgument("command_topic", default_value="~/l20_command"),
            DeclareLaunchArgument(
                "config_path",
                default_value="",
                description="Optional config YAML override. Empty uses package source default.",
            ),
            Node(
                package="manus_l20_revo_style",
                executable="revo_style_l20_node",
                name="manus_l20_revo_style_right",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "config_path": LaunchConfiguration("config_path"),
                    }
                ],
            ),
        ]
    )
