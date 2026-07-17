from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("manus_l20_revo_style"), "config", "four_finger_direct_l20_right.yaml"]
    )
    command_topic = "/manus_l20_four_finger_direct_right/l20_command"
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/manus_glove_0"),
            DeclareLaunchArgument("enable_viewer", default_value="true"),
            Node(
                package="manus_l20_revo_style",
                executable="revo_style_l20_node",
                name="manus_l20_four_finger_direct_right",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "command_topic": command_topic,
                        "config_path": config_path,
                    }
                ],
            ),
            Node(
                package="manus_l20_revo_style",
                executable="revo_style_l20_mujoco_node",
                name="manus_l20_four_finger_direct_mujoco_right",
                output="screen",
                parameters=[
                    {
                        "command_topic": command_topic,
                        "enable_viewer": LaunchConfiguration("enable_viewer"),
                        "config_path": config_path,
                    }
                ],
            ),
        ]
    )
