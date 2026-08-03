from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("manus_l20_haptics")
    tactile_yaml = os.path.join(pkg_share, "config", "tactile_source.yaml")
    haptic_yaml = os.path.join(pkg_share, "config", "haptic_feedback.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("hand_type", default_value="left"),
            DeclareLaunchArgument("hand_joint", default_value="L20"),
            DeclareLaunchArgument("can_channel", default_value="can0"),
            DeclareLaunchArgument("glove_id", default_value="0"),
            DeclareLaunchArgument("force_topic", default_value="/manus_l20_haptics/force"),
            DeclareLaunchArgument("vib_topic", default_value=""),
            DeclareLaunchArgument("mock_tactile", default_value="false"),
            DeclareLaunchArgument("poll_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("read_mode", default_value="auto"),
            Node(
                package="manus_l20_haptics",
                executable="tactile_source_node",
                name="linkerhand_l20_tactile_source",
                output="screen",
                parameters=[
                    tactile_yaml,
                    {
                        "hand_type": LaunchConfiguration("hand_type"),
                        "hand_joint": LaunchConfiguration("hand_joint"),
                        "can_channel": LaunchConfiguration("can_channel"),
                        "force_topic": LaunchConfiguration("force_topic"),
                        "mock": LaunchConfiguration("mock_tactile"),
                        "poll_rate_hz": LaunchConfiguration("poll_rate_hz"),
                        "read_mode": LaunchConfiguration("read_mode"),
                    },
                ],
            ),
            Node(
                package="manus_l20_haptics",
                executable="haptic_feedback_node",
                name="manus_l20_haptic_feedback",
                output="screen",
                parameters=[
                    haptic_yaml,
                    {
                        "glove_id": LaunchConfiguration("glove_id"),
                        "force_topic": LaunchConfiguration("force_topic"),
                        "vib_topic": LaunchConfiguration("vib_topic"),
                    },
                ],
            ),
        ]
    )
