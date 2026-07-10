import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _workspace_root() -> Path:
    env_root = os.environ.get("MANUS_L20_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / "src" / "somehand-feature").exists() and (parent / "src" / "manus_l20_retarget").exists():
            return parent
        if parent.name == "install":
            candidate = parent.parent
            if (candidate / "src" / "somehand-feature").exists():
                return candidate
    raise RuntimeError("Cannot locate Manus_L20_retarget workspace root; set MANUS_L20_ROOT")


_ROOT = _workspace_root()
_SRC = _ROOT / "src"
_SOMEHAND = _SRC / "somehand-feature"


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hand_type", default_value="right"),
            DeclareLaunchArgument("can", default_value="can0"),
            DeclareLaunchArgument("is_touch", default_value="false"),
            DeclareLaunchArgument("driver_speed", default_value="50,50,50,50,50"),
            DeclareLaunchArgument("driver_command_hz", default_value="60.0"),
            DeclareLaunchArgument("driver_state_hz", default_value="10.0"),
            DeclareLaunchArgument("driver_can_sleep_ms", default_value="3.0"),
            DeclareLaunchArgument("start_manus", default_value="false"),
            DeclareLaunchArgument("mapping_mode", default_value="landmark_flexion"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("max_delta_per_cycle", default_value="255"),
            DeclareLaunchArgument("lowpass_alpha", default_value="1.0"),
            DeclareLaunchArgument("landmark_transform", default_value="pico_native_to_rh"),
            DeclareLaunchArgument("wrist_mode", default_value="estimate"),
            DeclareLaunchArgument("distal_mode", default_value="dip"),
            DeclareLaunchArgument("enable_finger_yaw", default_value="true"),
            DeclareLaunchArgument("enable_finger_yaw_mapping", default_value="true"),
            DeclareLaunchArgument(
                "finger_yaw_calibration_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manus_l20_retarget"), "config", "finger_yaw_right_calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument("finger_yaw_source", default_value="tip"),
            DeclareLaunchArgument("finger_yaw_command_gain", default_value="-500.0"),
            DeclareLaunchArgument("finger_yaw_max_delta", default_value="120"),
            DeclareLaunchArgument(
                "flexion_calibration_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manus_l20_retarget"), "config", "flexion_right_calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument("enable_thumb_flexion_mapping", default_value="true"),
            DeclareLaunchArgument(
                "thumb_flexion_mapping_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manus_l20_retarget"), "config", "thumb_flexion_mapping.yaml"]
                ),
            ),
            DeclareLaunchArgument("thumb_flexion_root_gamma", default_value="1.0"),
            DeclareLaunchArgument("thumb_flexion_tip_gamma", default_value="1.0"),
            DeclareLaunchArgument("enable_thumb_yaw", default_value="false"),
            DeclareLaunchArgument("enable_thumb_roll", default_value="false"),
            DeclareLaunchArgument("enable_thumb_ik", default_value="true"),
            DeclareLaunchArgument("thumb_ik_mode", default_value="segment"),
            DeclareLaunchArgument("thumb_ik_debug", default_value="false"),
            DeclareLaunchArgument("thumb_segment_start", default_value="2"),
            DeclareLaunchArgument("thumb_segment_end", default_value="3"),
            DeclareLaunchArgument("thumb_segment_map_mode", default_value="raw"),
            DeclareLaunchArgument("thumb_segment_align_open", default_value="true"),
            DeclareLaunchArgument("thumb_segment_open_calibration_sec", default_value="1.0"),
            DeclareLaunchArgument("thumb_segment_manus_open_vector", default_value=""),
            DeclareLaunchArgument(
                "thumb_segment_manus_open_vector_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("manus_l20_retarget"), "config", "thumb_segment_open_vector_right.yaml"]
                ),
            ),
            DeclareLaunchArgument("thumb_segment_scale", default_value="1.0"),
            DeclareLaunchArgument("thumb_segment_damping", default_value="0.0008"),
            DeclareLaunchArgument("thumb_segment_max_step", default_value="0.20"),
            DeclareLaunchArgument("thumb_robot_segment_body", default_value="thumb_metacarpals"),
            DeclareLaunchArgument(
                "thumb_segment_robot_open_command",
                default_value="[254,248,246,249,254,128,115,111,131,188,221,255,255,255,255,254,254,254,254,254]",
            ),
            DeclareLaunchArgument("thumb_output_smoothing", default_value="false"),
            DeclareLaunchArgument("thumb_segment_roll_command_scale", default_value="1.0"),
            DeclareLaunchArgument("thumb_segment_yaw_command_scale", default_value="1.0"),
            DeclareLaunchArgument("thumb_segment_roll_command_deadzone", default_value="0"),
            DeclareLaunchArgument("thumb_segment_roll_command_gamma", default_value="1.0"),
            DeclareLaunchArgument("thumb_yaw_command_gain", default_value="-180.0"),
            DeclareLaunchArgument("thumb_roll_command_gain", default_value="-160.0"),
            DeclareLaunchArgument("thumb_yaw_max_delta", default_value="80"),
            DeclareLaunchArgument("thumb_roll_max_delta", default_value="80"),
            DeclareLaunchArgument("somehand_root", default_value=str(_SOMEHAND)),
            DeclareLaunchArgument(
                "somehand_config_path",
                default_value=str(_SOMEHAND / "configs" / "retargeting" / "right" / "linkerhand_l20_right.yaml"),
            ),
            DeclareLaunchArgument(
                "linkerhand_sdk_root",
                default_value=str(_SOMEHAND / "third_party" / "linkerhand-python-sdk"),
            ),
            Node(
                package="manus_l20_retarget",
                executable="manus_somehand_retarget_node",
                name="manus_somehand_retarget",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/manus_glove_0",
                        "command_topic": ["/cb_", LaunchConfiguration("hand_type"), "_hand_control_cmd"],
                        "somehand_root": LaunchConfiguration("somehand_root"),
                        "somehand_config_path": LaunchConfiguration("somehand_config_path"),
                        "linkerhand_sdk_root": LaunchConfiguration("linkerhand_sdk_root"),
                        "hand_family": "L20",
                        "mapping_mode": LaunchConfiguration("mapping_mode"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "max_delta_per_cycle": LaunchConfiguration("max_delta_per_cycle"),
                        "lowpass_alpha": LaunchConfiguration("lowpass_alpha"),
                        "landmark_transform": LaunchConfiguration("landmark_transform"),
                        "wrist_mode": LaunchConfiguration("wrist_mode"),
                        "distal_mode": LaunchConfiguration("distal_mode"),
                        "enable_finger_yaw": LaunchConfiguration("enable_finger_yaw"),
                        "enable_finger_yaw_mapping": LaunchConfiguration("enable_finger_yaw_mapping"),
                        "finger_yaw_calibration_path": LaunchConfiguration("finger_yaw_calibration_path"),
                        "finger_yaw_source": LaunchConfiguration("finger_yaw_source"),
                        "finger_yaw_command_gain": LaunchConfiguration("finger_yaw_command_gain"),
                        "finger_yaw_max_delta": LaunchConfiguration("finger_yaw_max_delta"),
                        "flexion_calibration_path": LaunchConfiguration("flexion_calibration_path"),
                        "enable_thumb_flexion_mapping": LaunchConfiguration("enable_thumb_flexion_mapping"),
                        "thumb_flexion_mapping_path": LaunchConfiguration("thumb_flexion_mapping_path"),
                        "thumb_flexion_root_gamma": LaunchConfiguration("thumb_flexion_root_gamma"),
                        "thumb_flexion_tip_gamma": LaunchConfiguration("thumb_flexion_tip_gamma"),
                        "enable_thumb_yaw": LaunchConfiguration("enable_thumb_yaw"),
                        "enable_thumb_roll": LaunchConfiguration("enable_thumb_roll"),
                        "enable_thumb_ik": LaunchConfiguration("enable_thumb_ik"),
                        "thumb_ik_mode": LaunchConfiguration("thumb_ik_mode"),
                        "thumb_ik_debug": LaunchConfiguration("thumb_ik_debug"),
                        "thumb_segment_start": LaunchConfiguration("thumb_segment_start"),
                        "thumb_segment_end": LaunchConfiguration("thumb_segment_end"),
                        "thumb_segment_map_mode": LaunchConfiguration("thumb_segment_map_mode"),
                        "thumb_segment_align_open": LaunchConfiguration("thumb_segment_align_open"),
                        "thumb_segment_open_calibration_sec": LaunchConfiguration("thumb_segment_open_calibration_sec"),
                        "thumb_segment_manus_open_vector": LaunchConfiguration("thumb_segment_manus_open_vector"),
                        "thumb_segment_manus_open_vector_path": LaunchConfiguration("thumb_segment_manus_open_vector_path"),
                        "thumb_segment_scale": LaunchConfiguration("thumb_segment_scale"),
                        "thumb_segment_damping": LaunchConfiguration("thumb_segment_damping"),
                        "thumb_segment_max_step": LaunchConfiguration("thumb_segment_max_step"),
                        "thumb_robot_segment_body": LaunchConfiguration("thumb_robot_segment_body"),
                        "thumb_segment_robot_open_command": LaunchConfiguration("thumb_segment_robot_open_command"),
                        "thumb_output_smoothing": LaunchConfiguration("thumb_output_smoothing"),
                        "thumb_segment_roll_command_scale": LaunchConfiguration("thumb_segment_roll_command_scale"),
                        "thumb_segment_yaw_command_scale": LaunchConfiguration("thumb_segment_yaw_command_scale"),
                        "thumb_segment_roll_command_deadzone": LaunchConfiguration("thumb_segment_roll_command_deadzone"),
                        "thumb_segment_roll_command_gamma": LaunchConfiguration("thumb_segment_roll_command_gamma"),
                        "thumb_yaw_command_gain": LaunchConfiguration("thumb_yaw_command_gain"),
                        "thumb_roll_command_gain": LaunchConfiguration("thumb_roll_command_gain"),
                        "thumb_yaw_max_delta": LaunchConfiguration("thumb_yaw_max_delta"),
                        "thumb_roll_max_delta": LaunchConfiguration("thumb_roll_max_delta"),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "linker_hand_ros2_sdk",
                    "linker_hand_advanced_g20",
                    "--hand_type",
                    LaunchConfiguration("hand_type"),
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
                ],
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration("start_manus")),
                cmd=["ros2", "run", "manus_ros2", "manus_data_publisher"],
                output="screen",
            ),
        ]
    )
