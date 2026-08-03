from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root() -> Path:
    env_root = os.environ.get("MANUS_L20_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()
    path = Path(__file__).resolve()
    for parent in path.parents:
        candidate = parent.parent if parent.name in {"src", "install"} else parent
        if (candidate / "src" / "l20_thumb_ik").exists() and (candidate / "src" / "manus_l20_retarget").exists():
            return candidate
        if (parent / "src" / "l20_thumb_ik").exists() and (parent / "src" / "manus_l20_retarget").exists():
            return parent
        if parent.name == "install":
            candidate = parent.parent
            if (candidate / "src" / "l20_thumb_ik").exists():
                return candidate
    raise RuntimeError("Cannot locate Manus_L20_retarget workspace root; set MANUS_L20_ROOT")


_ROOT = _workspace_root()
_SRC = _ROOT / "src"
_THUMB_IK_ROOT = _SRC / "l20_thumb_ik"


def _config_file(name: str) -> str:
    return str(_SRC / "manus_l20_retarget" / "config" / name)


def _thumb_segment_robot_open_command(hand_type: str) -> str:
    return "[254,248,246,249,254,128,115,111,131,188,221,255,255,255,255,254,254,254,254,254]"


def _finger_flexion_ergonomics_config_name(hand_type: str) -> str:
    return f"finger_flexion_ergonomics_{hand_type}_calibration.yaml"


def _finger_yaw_ergonomics_config_name(hand_type: str) -> str:
    return (
        "finger_yaw_ergonomics_left_calibration.yaml"
        if hand_type == "left"
        else "finger_yaw_ergonomics_right_calibration.yaml"
    )


def _finger_yaw_calibration_file(hand_type: str) -> str:
    return _config_file(_finger_yaw_ergonomics_config_name(hand_type))


def _thumb_flexion_ergonomics_config_name(hand_type: str) -> str:
    return f"thumb_{hand_type}_flexion_ergonomics_mapping.yaml"


def _fingertip_contact_semantics_default(hand_type: str) -> str:
    return _config_file(f"fingertip_contact_semantics_{hand_type}.yaml")


def _manus_calibration_file(hand_type: str) -> str:
    return str(_SRC / "manus_ros2" / "calibration" / f"Calibration_{hand_type}.mcal")


def _thumb_segment_frame_default(hand_type: str):
    return _config_file(f"thumb_segment_frame_{hand_type}.yaml")


def _hand_actions(
    *,
    hand_type: str,
    retarget_type: str,
    retarget_node_name: str,
    input_topic,
    can,
    is_touch,
    driver_speed,
    driver_command_hz,
    driver_state_hz,
    driver_can_sleep_ms,
    publish_rate_hz,
    max_delta_per_cycle,
    lowpass_alpha,
    landmark_transform,
    root_gamma,
    tip_gamma,
    finger_yaw_calibration_path,
    finger_flexion_ergonomics_calibration_path,
    thumb_flexion_ergonomics_mapping_path,
    thumb_flexion_root_gamma,
    thumb_flexion_tip_gamma,
    thumb_ik_debug,
    enable_fingertip_contact_semantics,
    fingertip_contact_semantics_path,
    fingertip_contact_debug,
    fingertip_contact_close_orientation_completion,
    fingertip_contact_close_flexion_start,
    fingertip_contact_release_flexion_open_completion,
    fingertip_contact_release_orientation_gamma,
    thumb_segment_start,
    thumb_segment_end,
    thumb_segment_frame_path,
    thumb_segment_scale,
    thumb_segment_damping,
    thumb_segment_max_step,
    thumb_robot_segment_body,
    thumb_segment_robot_open_command,
    thumb_segment_roll_command_scale,
    thumb_segment_yaw_command_scale,
    thumb_segment_roll_command_deadzone,
    thumb_segment_roll_command_gamma,
    thumb_segment_roll_progress_gate_start,
    thumb_segment_roll_progress_gate_end,
    thumb_segment_yaw_progress_gate_start,
    thumb_segment_yaw_progress_gate_end,
    l20_thumb_ik_root,
    l20_thumb_ik_config_path,
    linkerhand_sdk_root,
    enable_haptics,
    mock_tactile,
    haptic_glove_id,
    haptic_force_topic,
    haptic_vib_topic,
    haptic_poll_rate_hz,
    haptic_read_mode,
    haptic_normal_force_full_scale,
):
    if hand_type not in {"left", "right"}:
        raise ValueError(f"hand_type must be left or right, got {hand_type!r}")
    if retarget_type not in {"left", "right"}:
        raise ValueError(f"retarget_type must be left or right, got {retarget_type!r}")

    return [
        Node(
            package="manus_l20_retarget",
            executable="manus_l20_retarget_node",
            name=retarget_node_name,
            output="screen",
            parameters=[
                {
                    "input_topic": input_topic,
                    "command_topic": f"/cb_{hand_type}_hand_control_cmd",
                    "l20_thumb_ik_root": l20_thumb_ik_root,
                    "l20_thumb_ik_config_path": l20_thumb_ik_config_path,
                    "linkerhand_sdk_root": linkerhand_sdk_root,
                    "hand_family": "L20",
                    "publish_rate_hz": publish_rate_hz,
                    "max_delta_per_cycle": max_delta_per_cycle,
                    "lowpass_alpha": lowpass_alpha,
                    "landmark_transform": landmark_transform,
                    "root_gamma": root_gamma,
                    "tip_gamma": tip_gamma,
                    "finger_yaw_calibration_path": finger_yaw_calibration_path,
                    "finger_flexion_ergonomics_calibration_path": finger_flexion_ergonomics_calibration_path,
                    "thumb_flexion_ergonomics_mapping_path": thumb_flexion_ergonomics_mapping_path,
                    "thumb_flexion_root_gamma": thumb_flexion_root_gamma,
                    "thumb_flexion_tip_gamma": thumb_flexion_tip_gamma,
                    "thumb_ik_debug": thumb_ik_debug,
                    "enable_fingertip_contact_semantics": enable_fingertip_contact_semantics,
                    "fingertip_contact_semantics_path": fingertip_contact_semantics_path,
                    "fingertip_contact_debug": fingertip_contact_debug,
                    "fingertip_contact_close_orientation_completion": fingertip_contact_close_orientation_completion,
                    "fingertip_contact_close_flexion_start": fingertip_contact_close_flexion_start,
                    "fingertip_contact_release_flexion_open_completion": fingertip_contact_release_flexion_open_completion,
                    "fingertip_contact_release_orientation_gamma": fingertip_contact_release_orientation_gamma,
                    "thumb_segment_start": thumb_segment_start,
                    "thumb_segment_end": thumb_segment_end,
                    "thumb_segment_frame_path": thumb_segment_frame_path,
                    "thumb_segment_scale": thumb_segment_scale,
                    "thumb_segment_damping": thumb_segment_damping,
                    "thumb_segment_max_step": thumb_segment_max_step,
                    "thumb_robot_segment_body": thumb_robot_segment_body,
                    "thumb_segment_robot_open_command": thumb_segment_robot_open_command,
                    "thumb_segment_roll_command_scale": thumb_segment_roll_command_scale,
                    "thumb_segment_yaw_command_scale": thumb_segment_yaw_command_scale,
                    "thumb_segment_roll_command_deadzone": thumb_segment_roll_command_deadzone,
                    "thumb_segment_roll_command_gamma": thumb_segment_roll_command_gamma,
                    "thumb_segment_roll_progress_gate_start": thumb_segment_roll_progress_gate_start,
                    "thumb_segment_roll_progress_gate_end": thumb_segment_roll_progress_gate_end,
                    "thumb_segment_yaw_progress_gate_start": thumb_segment_yaw_progress_gate_start,
                    "thumb_segment_yaw_progress_gate_end": thumb_segment_yaw_progress_gate_end,
                }
            ],
        ),
        Node(
            package="manus_l20_haptics",
            executable="tactile_source_node",
            name=f"linkerhand_l20_tactile_source_{hand_type}",
            output="screen",
            condition=IfCondition(enable_haptics),
            parameters=[
                {
                    "enabled": True,
                    "mock": mock_tactile,
                    "hand_joint": "G20",
                    "hand_type": hand_type,
                    "can_channel": can,
                    "poll_rate_hz": haptic_poll_rate_hz,
                    "force_topic": haptic_force_topic,
                    "read_mode": haptic_read_mode,
                }
            ],
        ),
        Node(
            package="manus_l20_haptics",
            executable="haptic_feedback_node",
            name=f"manus_l20_haptic_feedback_{hand_type}",
            output="screen",
            condition=IfCondition(enable_haptics),
            parameters=[
                {
                    "glove_id": haptic_glove_id,
                    "force_topic": haptic_force_topic,
                    "vib_topic": haptic_vib_topic,
                    "normal_force_full_scale": haptic_normal_force_full_scale,
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
                hand_type,
                "--can",
                can,
                "--is_touch",
                is_touch,
                "--speed",
                driver_speed,
                "--command_hz",
                driver_command_hz,
                "--state_hz",
                driver_state_hz,
                "--can_sleep_ms",
                driver_can_sleep_ms,
                "--ros-args",
                "-r",
                f"__node:=linker_hand_advanced_g20_{hand_type}",
            ],
            output="screen",
        ),
    ]


def generate_manus_l20_launch(
    *,
    hand_type: str,
    retarget_node_name: str,
    logic_hand_type: str | None = None,
) -> LaunchDescription:
    if hand_type not in {"left", "right"}:
        raise ValueError(f"hand_type must be left or right, got {hand_type!r}")
    logic_type = logic_hand_type or hand_type
    if logic_type not in {"left", "right"}:
        raise ValueError(f"logic_hand_type must be left or right, got {logic_type!r}")
    # The left-hand runtime intentionally reuses the verified right-hand
    # retargeting path and only switches the physical driver topic/hand ID.
    retarget_type = "right"
    landmark_transform_default = "left_glove_to_right_retarget" if logic_type == "left" else "right_glove_to_right_retarget"

    return LaunchDescription(
        [
            DeclareLaunchArgument("can", default_value="can0"),
            DeclareLaunchArgument("is_touch", default_value="false"),
            DeclareLaunchArgument("input_topic", default_value="/manus_glove_0"),
            DeclareLaunchArgument("driver_speed", default_value="80,80,80,80,80"),
            DeclareLaunchArgument("driver_command_hz", default_value="200.0"),
            DeclareLaunchArgument("driver_state_hz", default_value="10.0"),
            DeclareLaunchArgument("driver_can_sleep_ms", default_value="3.0"),
            DeclareLaunchArgument("start_manus", default_value="false"),
            DeclareLaunchArgument("load_manus_calibration", default_value="true"),
            DeclareLaunchArgument("left_manus_calibration_path", default_value=_manus_calibration_file("left")),
            DeclareLaunchArgument("right_manus_calibration_path", default_value=_manus_calibration_file("right")),
            DeclareLaunchArgument("enable_haptics", default_value="false"),
            DeclareLaunchArgument("mock_tactile", default_value="false"),
            DeclareLaunchArgument("haptic_glove_id", default_value="0"),
            DeclareLaunchArgument("haptic_force_topic", default_value=f"/manus_l20_haptics/{hand_type}/force"),
            DeclareLaunchArgument("haptic_vib_topic", default_value=""),
            DeclareLaunchArgument("haptic_poll_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("haptic_read_mode", default_value="auto"),
            DeclareLaunchArgument("haptic_normal_force_full_scale", default_value="100.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="120.0"),
            DeclareLaunchArgument("max_delta_per_cycle", default_value="255"),
            DeclareLaunchArgument("lowpass_alpha", default_value="1.0"),
            DeclareLaunchArgument("landmark_transform", default_value=landmark_transform_default),
            DeclareLaunchArgument("root_gamma", default_value="0.2"),
            DeclareLaunchArgument("tip_gamma", default_value="1.0"),
            DeclareLaunchArgument(
                "finger_yaw_calibration_path",
                default_value=_finger_yaw_calibration_file(logic_type),
            ),
            DeclareLaunchArgument(
                "finger_flexion_ergonomics_calibration_path",
                default_value=_config_file(_finger_flexion_ergonomics_config_name(logic_type)),
            ),
            DeclareLaunchArgument(
                "thumb_flexion_ergonomics_mapping_path",
                default_value=_config_file(_thumb_flexion_ergonomics_config_name(logic_type)),
            ),
            DeclareLaunchArgument("thumb_flexion_root_gamma", default_value="1.0"),
            DeclareLaunchArgument("thumb_flexion_tip_gamma", default_value="1.0"),
            DeclareLaunchArgument("thumb_ik_debug", default_value="false"),
            DeclareLaunchArgument("enable_fingertip_contact_semantics", default_value="true"),
            DeclareLaunchArgument(
                "fingertip_contact_semantics_path",
                default_value=_fingertip_contact_semantics_default(logic_type),
            ),
            DeclareLaunchArgument("fingertip_contact_debug", default_value="false"),
            DeclareLaunchArgument("fingertip_contact_close_orientation_completion", default_value="0.25"),
            DeclareLaunchArgument("fingertip_contact_close_flexion_start", default_value="0.40"),
            DeclareLaunchArgument("fingertip_contact_release_flexion_open_completion", default_value="0.18"),
            DeclareLaunchArgument("fingertip_contact_release_orientation_gamma", default_value="2.5"),
            DeclareLaunchArgument("thumb_segment_start", default_value="2"),
            DeclareLaunchArgument("thumb_segment_end", default_value="3"),
            DeclareLaunchArgument("thumb_segment_frame_path", default_value=_thumb_segment_frame_default(logic_type)),
            DeclareLaunchArgument("thumb_segment_scale", default_value="1.0"),
            DeclareLaunchArgument("thumb_segment_damping", default_value="0.0008"),
            DeclareLaunchArgument("thumb_segment_max_step", default_value="0.20"),
            DeclareLaunchArgument("thumb_robot_segment_body", default_value="thumb_metacarpals"),
            DeclareLaunchArgument(
                "thumb_segment_robot_open_command",
                default_value=_thumb_segment_robot_open_command(hand_type),
            ),
            DeclareLaunchArgument("thumb_segment_roll_command_scale", default_value="1.0"),
            DeclareLaunchArgument("thumb_segment_yaw_command_scale", default_value="1.0"),
            DeclareLaunchArgument("thumb_segment_roll_command_deadzone", default_value="0"),
            DeclareLaunchArgument("thumb_segment_roll_command_gamma", default_value="1.0"),
            DeclareLaunchArgument(
                "thumb_segment_roll_progress_gate_start",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "thumb_segment_roll_progress_gate_end",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "thumb_segment_yaw_progress_gate_start",
                default_value="0.25" if logic_type == "left" else "0.0",
            ),
            DeclareLaunchArgument("thumb_segment_yaw_progress_gate_end", default_value="1.0"),
            DeclareLaunchArgument("l20_thumb_ik_root", default_value=str(_THUMB_IK_ROOT)),
            DeclareLaunchArgument(
                "l20_thumb_ik_config_path",
                default_value=str(_THUMB_IK_ROOT / "configs" / "retargeting" / retarget_type / f"linkerhand_l20_{retarget_type}.yaml"),
            ),
            DeclareLaunchArgument(
                "linkerhand_sdk_root",
                default_value=str(_THUMB_IK_ROOT / "third_party" / "linkerhand-python-sdk"),
            ),
            *_hand_actions(
                hand_type=hand_type,
                retarget_type=retarget_type,
                retarget_node_name=retarget_node_name,
                input_topic=LaunchConfiguration("input_topic"),
                can=LaunchConfiguration("can"),
                is_touch=LaunchConfiguration("is_touch"),
                driver_speed=LaunchConfiguration("driver_speed"),
                driver_command_hz=LaunchConfiguration("driver_command_hz"),
                driver_state_hz=LaunchConfiguration("driver_state_hz"),
                driver_can_sleep_ms=LaunchConfiguration("driver_can_sleep_ms"),
                publish_rate_hz=LaunchConfiguration("publish_rate_hz"),
                max_delta_per_cycle=LaunchConfiguration("max_delta_per_cycle"),
                lowpass_alpha=LaunchConfiguration("lowpass_alpha"),
                landmark_transform=LaunchConfiguration("landmark_transform"),
                root_gamma=LaunchConfiguration("root_gamma"),
                tip_gamma=LaunchConfiguration("tip_gamma"),
                finger_yaw_calibration_path=LaunchConfiguration("finger_yaw_calibration_path"),
                finger_flexion_ergonomics_calibration_path=LaunchConfiguration(
                    "finger_flexion_ergonomics_calibration_path"
                ),
                thumb_flexion_ergonomics_mapping_path=LaunchConfiguration(
                    "thumb_flexion_ergonomics_mapping_path"
                ),
                thumb_flexion_root_gamma=LaunchConfiguration("thumb_flexion_root_gamma"),
                thumb_flexion_tip_gamma=LaunchConfiguration("thumb_flexion_tip_gamma"),
                thumb_ik_debug=LaunchConfiguration("thumb_ik_debug"),
                enable_fingertip_contact_semantics=LaunchConfiguration("enable_fingertip_contact_semantics"),
                fingertip_contact_semantics_path=LaunchConfiguration("fingertip_contact_semantics_path"),
                fingertip_contact_debug=LaunchConfiguration("fingertip_contact_debug"),
                fingertip_contact_close_orientation_completion=LaunchConfiguration(
                    "fingertip_contact_close_orientation_completion"
                ),
                fingertip_contact_close_flexion_start=LaunchConfiguration("fingertip_contact_close_flexion_start"),
                fingertip_contact_release_flexion_open_completion=LaunchConfiguration(
                    "fingertip_contact_release_flexion_open_completion"
                ),
                fingertip_contact_release_orientation_gamma=LaunchConfiguration(
                    "fingertip_contact_release_orientation_gamma"
                ),
                thumb_segment_start=LaunchConfiguration("thumb_segment_start"),
                thumb_segment_end=LaunchConfiguration("thumb_segment_end"),
                thumb_segment_frame_path=LaunchConfiguration("thumb_segment_frame_path"),
                thumb_segment_scale=LaunchConfiguration("thumb_segment_scale"),
                thumb_segment_damping=LaunchConfiguration("thumb_segment_damping"),
                thumb_segment_max_step=LaunchConfiguration("thumb_segment_max_step"),
                thumb_robot_segment_body=LaunchConfiguration("thumb_robot_segment_body"),
                thumb_segment_robot_open_command=LaunchConfiguration("thumb_segment_robot_open_command"),
                thumb_segment_roll_command_scale=LaunchConfiguration("thumb_segment_roll_command_scale"),
                thumb_segment_yaw_command_scale=LaunchConfiguration("thumb_segment_yaw_command_scale"),
                thumb_segment_roll_command_deadzone=LaunchConfiguration("thumb_segment_roll_command_deadzone"),
                thumb_segment_roll_command_gamma=LaunchConfiguration("thumb_segment_roll_command_gamma"),
                thumb_segment_roll_progress_gate_start=LaunchConfiguration("thumb_segment_roll_progress_gate_start"),
                thumb_segment_roll_progress_gate_end=LaunchConfiguration("thumb_segment_roll_progress_gate_end"),
                thumb_segment_yaw_progress_gate_start=LaunchConfiguration("thumb_segment_yaw_progress_gate_start"),
                thumb_segment_yaw_progress_gate_end=LaunchConfiguration("thumb_segment_yaw_progress_gate_end"),
                l20_thumb_ik_root=LaunchConfiguration("l20_thumb_ik_root"),
                l20_thumb_ik_config_path=LaunchConfiguration("l20_thumb_ik_config_path"),
                linkerhand_sdk_root=LaunchConfiguration("linkerhand_sdk_root"),
                enable_haptics=LaunchConfiguration("enable_haptics"),
                mock_tactile=LaunchConfiguration("mock_tactile"),
                haptic_glove_id=LaunchConfiguration("haptic_glove_id"),
                haptic_force_topic=LaunchConfiguration("haptic_force_topic"),
                haptic_vib_topic=LaunchConfiguration("haptic_vib_topic"),
                haptic_poll_rate_hz=LaunchConfiguration("haptic_poll_rate_hz"),
                haptic_read_mode=LaunchConfiguration("haptic_read_mode"),
                haptic_normal_force_full_scale=LaunchConfiguration("haptic_normal_force_full_scale"),
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_manus")),
                package="manus_ros2",
                executable="manus_data_publisher",
                name="manus_data_publisher",
                output="screen",
                parameters=[
                    {
                        "load_calibration": ParameterValue(LaunchConfiguration("load_manus_calibration"), value_type=bool),
                        "left_calibration_path": LaunchConfiguration("left_manus_calibration_path"),
                        "right_calibration_path": LaunchConfiguration("right_manus_calibration_path"),
                    }
                ],
            ),
        ]
    )


def generate_manus_l20_bimanual_launch() -> LaunchDescription:
    common_arguments = [
        DeclareLaunchArgument("right_can", default_value="can0"),
        DeclareLaunchArgument("left_can", default_value="can0"),
        DeclareLaunchArgument("is_touch", default_value="false"),
        DeclareLaunchArgument("right_input_topic", default_value="/manus_glove_0"),
        DeclareLaunchArgument("left_input_topic", default_value="/manus_glove_1"),
        DeclareLaunchArgument("driver_speed", default_value="80,80,80,80,80"),
        DeclareLaunchArgument("driver_command_hz", default_value="200.0"),
        DeclareLaunchArgument("driver_state_hz", default_value="10.0"),
        DeclareLaunchArgument("driver_can_sleep_ms", default_value="3.0"),
        DeclareLaunchArgument("start_manus", default_value="false"),
        DeclareLaunchArgument("load_manus_calibration", default_value="true"),
        DeclareLaunchArgument("left_manus_calibration_path", default_value=_manus_calibration_file("left")),
        DeclareLaunchArgument("right_manus_calibration_path", default_value=_manus_calibration_file("right")),
        DeclareLaunchArgument("enable_haptics", default_value="false"),
        DeclareLaunchArgument("mock_tactile", default_value="false"),
        DeclareLaunchArgument("right_haptic_glove_id", default_value="0"),
        DeclareLaunchArgument("left_haptic_glove_id", default_value="1"),
        DeclareLaunchArgument("right_haptic_force_topic", default_value="/manus_l20_haptics/right/force"),
        DeclareLaunchArgument("left_haptic_force_topic", default_value="/manus_l20_haptics/left/force"),
        DeclareLaunchArgument("right_haptic_vib_topic", default_value=""),
        DeclareLaunchArgument("left_haptic_vib_topic", default_value=""),
        DeclareLaunchArgument("haptic_poll_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("haptic_read_mode", default_value="auto"),
        DeclareLaunchArgument("haptic_normal_force_full_scale", default_value="100.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="120.0"),
        DeclareLaunchArgument("max_delta_per_cycle", default_value="255"),
        DeclareLaunchArgument("lowpass_alpha", default_value="1.0"),
        DeclareLaunchArgument("right_landmark_transform", default_value="right_glove_to_right_retarget"),
        DeclareLaunchArgument("left_landmark_transform", default_value="left_glove_to_right_retarget"),
        DeclareLaunchArgument("root_gamma", default_value="0.2"),
        DeclareLaunchArgument("tip_gamma", default_value="1.0"),
        DeclareLaunchArgument(
            "right_finger_flexion_ergonomics_calibration_path",
            default_value=_config_file(_finger_flexion_ergonomics_config_name("right")),
        ),
        DeclareLaunchArgument(
            "left_finger_flexion_ergonomics_calibration_path",
            default_value=_config_file(_finger_flexion_ergonomics_config_name("left")),
        ),
        DeclareLaunchArgument("right_finger_yaw_calibration_path", default_value=_finger_yaw_calibration_file("right")),
        DeclareLaunchArgument("left_finger_yaw_calibration_path", default_value=_finger_yaw_calibration_file("left")),
        DeclareLaunchArgument(
            "right_thumb_flexion_ergonomics_mapping_path",
            default_value=_config_file(_thumb_flexion_ergonomics_config_name("right")),
        ),
        DeclareLaunchArgument(
            "left_thumb_flexion_ergonomics_mapping_path",
            default_value=_config_file(_thumb_flexion_ergonomics_config_name("left")),
        ),
        DeclareLaunchArgument(
            "right_fingertip_contact_semantics_path",
            default_value=_fingertip_contact_semantics_default("right"),
        ),
        DeclareLaunchArgument(
            "left_fingertip_contact_semantics_path",
            default_value=_fingertip_contact_semantics_default("left"),
        ),
        DeclareLaunchArgument("thumb_flexion_root_gamma", default_value="1.0"),
        DeclareLaunchArgument("thumb_flexion_tip_gamma", default_value="1.0"),
        DeclareLaunchArgument("thumb_ik_debug", default_value="false"),
        DeclareLaunchArgument("enable_fingertip_contact_semantics", default_value="true"),
        DeclareLaunchArgument("fingertip_contact_debug", default_value="false"),
        DeclareLaunchArgument("fingertip_contact_close_orientation_completion", default_value="0.25"),
        DeclareLaunchArgument("fingertip_contact_close_flexion_start", default_value="0.40"),
        DeclareLaunchArgument("fingertip_contact_release_flexion_open_completion", default_value="0.18"),
        DeclareLaunchArgument("fingertip_contact_release_orientation_gamma", default_value="2.5"),
        DeclareLaunchArgument("thumb_segment_start", default_value="2"),
        DeclareLaunchArgument("thumb_segment_end", default_value="3"),
        DeclareLaunchArgument("thumb_segment_scale", default_value="1.0"),
        DeclareLaunchArgument("thumb_segment_damping", default_value="0.0008"),
        DeclareLaunchArgument("thumb_segment_max_step", default_value="0.20"),
        DeclareLaunchArgument("thumb_robot_segment_body", default_value="thumb_metacarpals"),
        DeclareLaunchArgument(
            "thumb_segment_robot_open_command",
            default_value=_thumb_segment_robot_open_command("right"),
        ),
        DeclareLaunchArgument("thumb_segment_roll_command_scale", default_value="1.0"),
        DeclareLaunchArgument("thumb_segment_yaw_command_scale", default_value="1.0"),
        DeclareLaunchArgument("thumb_segment_roll_command_deadzone", default_value="0"),
        DeclareLaunchArgument("thumb_segment_roll_command_gamma", default_value="1.0"),
        DeclareLaunchArgument("right_thumb_segment_roll_progress_gate_start", default_value="0.0"),
        DeclareLaunchArgument("right_thumb_segment_roll_progress_gate_end", default_value="1.0"),
        DeclareLaunchArgument("left_thumb_segment_roll_progress_gate_start", default_value="0.0"),
        DeclareLaunchArgument("left_thumb_segment_roll_progress_gate_end", default_value="1.0"),
        DeclareLaunchArgument("right_thumb_segment_yaw_progress_gate_start", default_value="0.0"),
        DeclareLaunchArgument("right_thumb_segment_yaw_progress_gate_end", default_value="1.0"),
        DeclareLaunchArgument("left_thumb_segment_yaw_progress_gate_start", default_value="0.25"),
        DeclareLaunchArgument("left_thumb_segment_yaw_progress_gate_end", default_value="1.0"),
        DeclareLaunchArgument("l20_thumb_ik_root", default_value=str(_THUMB_IK_ROOT)),
        DeclareLaunchArgument(
            "linkerhand_sdk_root",
            default_value=str(_THUMB_IK_ROOT / "third_party" / "linkerhand-python-sdk"),
        ),
    ]

    def hand_actions(
        hand_type: str,
        input_topic_name: str,
        can_name: str,
        transform_name: str,
        roll_gate_start_name: str,
        roll_gate_end_name: str,
        yaw_gate_start_name: str,
        yaw_gate_end_name: str,
        finger_flexion_ergonomics_calibration_path_name: str,
        finger_yaw_calibration_path_name: str,
        thumb_flexion_ergonomics_mapping_path_name: str,
        fingertip_contact_semantics_path_name: str,
        haptic_glove_id_name: str,
        haptic_force_topic_name: str,
        haptic_vib_topic_name: str,
    ):
        retarget_type = "right"
        return _hand_actions(
            hand_type=hand_type,
            retarget_type=retarget_type,
            retarget_node_name=f"manus_l20_retarget_{hand_type}",
            input_topic=LaunchConfiguration(input_topic_name),
            can=LaunchConfiguration(can_name),
            is_touch=LaunchConfiguration("is_touch"),
            driver_speed=LaunchConfiguration("driver_speed"),
            driver_command_hz=LaunchConfiguration("driver_command_hz"),
            driver_state_hz=LaunchConfiguration("driver_state_hz"),
            driver_can_sleep_ms=LaunchConfiguration("driver_can_sleep_ms"),
            publish_rate_hz=LaunchConfiguration("publish_rate_hz"),
            max_delta_per_cycle=LaunchConfiguration("max_delta_per_cycle"),
            lowpass_alpha=LaunchConfiguration("lowpass_alpha"),
            landmark_transform=LaunchConfiguration(transform_name),
            root_gamma=LaunchConfiguration("root_gamma"),
            tip_gamma=LaunchConfiguration("tip_gamma"),
            finger_yaw_calibration_path=LaunchConfiguration(finger_yaw_calibration_path_name),
            finger_flexion_ergonomics_calibration_path=LaunchConfiguration(
                finger_flexion_ergonomics_calibration_path_name
            ),
            thumb_flexion_ergonomics_mapping_path=LaunchConfiguration(
                thumb_flexion_ergonomics_mapping_path_name
            ),
            thumb_flexion_root_gamma=LaunchConfiguration("thumb_flexion_root_gamma"),
            thumb_flexion_tip_gamma=LaunchConfiguration("thumb_flexion_tip_gamma"),
            thumb_ik_debug=LaunchConfiguration("thumb_ik_debug"),
            enable_fingertip_contact_semantics=LaunchConfiguration("enable_fingertip_contact_semantics"),
            fingertip_contact_semantics_path=LaunchConfiguration(fingertip_contact_semantics_path_name),
            fingertip_contact_debug=LaunchConfiguration("fingertip_contact_debug"),
            fingertip_contact_close_orientation_completion=LaunchConfiguration(
                "fingertip_contact_close_orientation_completion"
            ),
            fingertip_contact_close_flexion_start=LaunchConfiguration("fingertip_contact_close_flexion_start"),
            fingertip_contact_release_flexion_open_completion=LaunchConfiguration(
                "fingertip_contact_release_flexion_open_completion"
            ),
            fingertip_contact_release_orientation_gamma=LaunchConfiguration(
                "fingertip_contact_release_orientation_gamma"
            ),
            thumb_segment_start=LaunchConfiguration("thumb_segment_start"),
            thumb_segment_end=LaunchConfiguration("thumb_segment_end"),
            thumb_segment_frame_path=_thumb_segment_frame_default(hand_type),
            thumb_segment_scale=LaunchConfiguration("thumb_segment_scale"),
            thumb_segment_damping=LaunchConfiguration("thumb_segment_damping"),
            thumb_segment_max_step=LaunchConfiguration("thumb_segment_max_step"),
            thumb_robot_segment_body=LaunchConfiguration("thumb_robot_segment_body"),
            thumb_segment_robot_open_command=LaunchConfiguration("thumb_segment_robot_open_command"),
            thumb_segment_roll_command_scale=LaunchConfiguration("thumb_segment_roll_command_scale"),
            thumb_segment_yaw_command_scale=LaunchConfiguration("thumb_segment_yaw_command_scale"),
            thumb_segment_roll_command_deadzone=LaunchConfiguration("thumb_segment_roll_command_deadzone"),
            thumb_segment_roll_command_gamma=LaunchConfiguration("thumb_segment_roll_command_gamma"),
            thumb_segment_roll_progress_gate_start=LaunchConfiguration(roll_gate_start_name),
            thumb_segment_roll_progress_gate_end=LaunchConfiguration(roll_gate_end_name),
            thumb_segment_yaw_progress_gate_start=LaunchConfiguration(yaw_gate_start_name),
            thumb_segment_yaw_progress_gate_end=LaunchConfiguration(yaw_gate_end_name),
            l20_thumb_ik_root=LaunchConfiguration("l20_thumb_ik_root"),
            l20_thumb_ik_config_path=str(
                _THUMB_IK_ROOT / "configs" / "retargeting" / retarget_type / f"linkerhand_l20_{retarget_type}.yaml"
            ),
            linkerhand_sdk_root=LaunchConfiguration("linkerhand_sdk_root"),
            enable_haptics=LaunchConfiguration("enable_haptics"),
            mock_tactile=LaunchConfiguration("mock_tactile"),
            haptic_glove_id=LaunchConfiguration(haptic_glove_id_name),
            haptic_force_topic=LaunchConfiguration(haptic_force_topic_name),
            haptic_vib_topic=LaunchConfiguration(haptic_vib_topic_name),
            haptic_poll_rate_hz=LaunchConfiguration("haptic_poll_rate_hz"),
            haptic_read_mode=LaunchConfiguration("haptic_read_mode"),
            haptic_normal_force_full_scale=LaunchConfiguration("haptic_normal_force_full_scale"),
        )

    return LaunchDescription(
        [
            *common_arguments,
            *hand_actions(
                "right",
                "right_input_topic",
                "right_can",
                "right_landmark_transform",
                "right_thumb_segment_roll_progress_gate_start",
                "right_thumb_segment_roll_progress_gate_end",
                "right_thumb_segment_yaw_progress_gate_start",
                "right_thumb_segment_yaw_progress_gate_end",
                "right_finger_flexion_ergonomics_calibration_path",
                "right_finger_yaw_calibration_path",
                "right_thumb_flexion_ergonomics_mapping_path",
                "right_fingertip_contact_semantics_path",
                "right_haptic_glove_id",
                "right_haptic_force_topic",
                "right_haptic_vib_topic",
            ),
            *hand_actions(
                "left",
                "left_input_topic",
                "left_can",
                "left_landmark_transform",
                "left_thumb_segment_roll_progress_gate_start",
                "left_thumb_segment_roll_progress_gate_end",
                "left_thumb_segment_yaw_progress_gate_start",
                "left_thumb_segment_yaw_progress_gate_end",
                "left_finger_flexion_ergonomics_calibration_path",
                "left_finger_yaw_calibration_path",
                "left_thumb_flexion_ergonomics_mapping_path",
                "left_fingertip_contact_semantics_path",
                "left_haptic_glove_id",
                "left_haptic_force_topic",
                "left_haptic_vib_topic",
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_manus")),
                package="manus_ros2",
                executable="manus_data_publisher",
                name="manus_data_publisher",
                output="screen",
                parameters=[
                    {
                        "load_calibration": ParameterValue(LaunchConfiguration("load_manus_calibration"), value_type=bool),
                        "left_calibration_path": LaunchConfiguration("left_manus_calibration_path"),
                        "right_calibration_path": LaunchConfiguration("right_manus_calibration_path"),
                    }
                ],
            ),
        ]
    )
