from __future__ import annotations

import sys
import os
import threading
import math
from pathlib import Path
from time import monotonic
from typing import Any

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .mapping import clamp_u8
from .manus_landmarks import (
    _finger_joint_orientation_yaw_rad,
    _finger_mcp_orientation_yaw_rad,
    _finger_yaw_rad,
    _palm_frame,
    _thumb_pose_features,
    manus_raw_nodes_to_mediapipe_landmarks,
)


def _default_workspace_root() -> Path:
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

STANDARD_OPEN_COMMAND = [
    255,
    255,
    255,
    255,
    255,
    128,
    183,
    145,
    91,
    90,
    240,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
]
STANDARD_FIST_COMMAND = [
    127,
    14,
    1,
    6,
    33,
    128,
    163,
    129,
    86,
    62,
    255,
    255,
    255,
    255,
    255,
    131,
    44,
    30,
    3,
    10,
]
DEFAULT_ROOT_OPEN_RAD = [0.00814, 0.21187, 0.22807, 0.17647, 0.19789]
DEFAULT_ROOT_CLOSED_RAD = [0.75, 2.00, 2.05, 2.00, 1.80]
DEFAULT_TIP_OPEN_RAD = [0.19885, 0.03296, 0.06118, 0.21235, 0.12662]
DEFAULT_TIP_CLOSED_RAD = [0.80, 1.30, 1.80, 1.50, 1.70]
DEFAULT_FINGER_YAW_OPEN_RAD = [-0.26256, -0.0946, -0.00355, 0.16442]
DEFAULT_THUMB_YAW_OPEN_RAD = -0.74873
DEFAULT_THUMB_ROLL_OPEN_RAD = 0.22278
THUMB_CALIBRATION_SAMPLES = [
    {
        "features": {
            "root": 0.01819,
            "tip": 0.52230,
            "yaw": -0.89262,
            "roll": 0.24705,
            "distance": 0.10954,
        },
        "command": {
            0: 255,
            5: 128,
            10: 255,
            15: 255,
        },
    },
    {
        "features": {
            "root": 0.00172,
            "tip": 0.02707,
            "yaw": -0.38026,
            "roll": 0.78143,
            "distance": 0.01597,
        },
        "command": {
            0: 140,
            5: 160,
            10: 83,
            15: 166,
        },
    },
    {
        "features": {
            "root": 0.00759,
            "tip": 1.16323,
            "yaw": 0.53146,
            "roll": 0.40790,
            "distance": 0.11180,
        },
        "command": {
            0: 56,
            5: 50,
            10: 174,
            15: 48,
        },
    },
]
THUMB_CALIBRATION_FEATURE_WEIGHTS = {
    "root": 0.15,
    "tip": 2.0,
    "yaw": 2.0,
    "roll": 0.8,
    "distance": 0.8,
}
THUMB_FEATURE_KEYS = ("root", "tip", "yaw", "roll", "distance")
THUMB_COMMAND_SLOTS = (0, 5, 10, 15)
THUMB_IK_COMMAND_SLOTS = (5, 10)
THUMB_OUTPUT_ALPHA = 0.55
THUMB_OUTPUT_DEADBAND = 1
THUMB_OUTPUT_MAX_DELTA = 28
MANUS_NOMINAL_THUMB_LENGTH = 0.09722
FINGER_LANDMARKS = (
    (1, 2, 3, 4),
    (5, 6, 7, 8),
    (9, 10, 11, 12),
    (13, 14, 15, 16),
    (17, 18, 19, 20),
)


class ManusSomehandRetargetNode(Node):
    """Retarget MANUS raw skeleton nodes through the somehand L20 pipeline."""

    def __init__(self) -> None:
        super().__init__("manus_somehand_retarget")
        workspace = _default_workspace_root()
        default_somehand_root = workspace / "src" / "somehand-feature"
        default_config = default_somehand_root / "configs" / "retargeting" / "right" / "linkerhand_l20_right.yaml"
        default_sdk_root = default_somehand_root / "third_party" / "linkerhand-python-sdk"

        self.declare_parameter("input_topic", "/manus_glove_0")
        self.declare_parameter("command_topic", "/cb_right_hand_control_cmd")
        self.declare_parameter("somehand_root", str(default_somehand_root))
        self.declare_parameter("somehand_config_path", str(default_config))
        self.declare_parameter("linkerhand_sdk_root", str(default_sdk_root))
        self.declare_parameter("hand_family", "L20")
        self.declare_parameter("mapping_mode", "landmark_flexion")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("max_delta_per_cycle", 8)
        self.declare_parameter("lowpass_alpha", 0.45)
        self.declare_parameter("watchdog_timeout_sec", 0.3)
        self.declare_parameter("reserved_command", 255)
        self.declare_parameter("start_from_open", True)
        self.declare_parameter("neutral_command", STANDARD_OPEN_COMMAND)
        self.declare_parameter("closed_command", STANDARD_FIST_COMMAND)
        self.declare_parameter("lock_neutral_slots", [5, 6, 7, 8, 9, 10, 11, 12, 13, 14])
        self.declare_parameter("root_flexion_open_rad", DEFAULT_ROOT_OPEN_RAD)
        self.declare_parameter("root_flexion_closed_rad", DEFAULT_ROOT_CLOSED_RAD)
        self.declare_parameter("tip_flexion_open_rad", DEFAULT_TIP_OPEN_RAD)
        self.declare_parameter("tip_flexion_closed_rad", DEFAULT_TIP_CLOSED_RAD)
        self.declare_parameter("flexion_calibration_path", "")
        self.declare_parameter("root_gamma", 1.0)
        self.declare_parameter("tip_gamma", 1.0)
        self.declare_parameter("enable_finger_yaw", False)
        self.declare_parameter("enable_finger_yaw_mapping", False)
        self.declare_parameter("finger_yaw_calibration_path", "")
        self.declare_parameter("finger_yaw_source", "tip")
        self.declare_parameter("finger_yaw_open_rad", DEFAULT_FINGER_YAW_OPEN_RAD)
        self.declare_parameter("finger_yaw_command_gain", -140.0)
        self.declare_parameter("finger_yaw_max_delta", 35)
        self.declare_parameter("enable_thumb_yaw", False)
        self.declare_parameter("enable_thumb_roll", False)
        self.declare_parameter("enable_thumb_flexion_mapping", False)
        self.declare_parameter("thumb_flexion_mapping_path", "")
        self.declare_parameter("thumb_flexion_root_gamma", 1.0)
        self.declare_parameter("thumb_flexion_tip_gamma", 1.0)
        self.declare_parameter("enable_thumb_ik", False)
        self.declare_parameter("thumb_ik_mode", "segment")
        self.declare_parameter("thumb_ik_debug", False)
        self.declare_parameter("thumb_segment_start", 2)
        self.declare_parameter("thumb_segment_end", 3)
        self.declare_parameter("thumb_segment_map_mode", "raw")
        self.declare_parameter("thumb_segment_align_open", True)
        self.declare_parameter("thumb_segment_open_calibration_sec", 1.0)
        self.declare_parameter("thumb_segment_manus_open_vector", "")
        self.declare_parameter("thumb_segment_manus_open_vector_path", "")
        self.declare_parameter("thumb_segment_scale", 1.0)
        self.declare_parameter("thumb_segment_damping", 8e-4)
        self.declare_parameter("thumb_segment_max_step", 0.20)
        self.declare_parameter("thumb_robot_segment_body", "thumb_metacarpals")
        self.declare_parameter("thumb_segment_robot_open_command", STANDARD_OPEN_COMMAND)
        self.declare_parameter("thumb_output_smoothing", True)
        self.declare_parameter("thumb_segment_roll_command_scale", 1.0)
        self.declare_parameter("thumb_segment_yaw_command_scale", 1.0)
        self.declare_parameter("thumb_segment_roll_command_deadzone", 0)
        self.declare_parameter("thumb_segment_roll_command_gamma", 1.0)
        self.declare_parameter("thumb_yaw_open_rad", DEFAULT_THUMB_YAW_OPEN_RAD)
        self.declare_parameter("thumb_roll_open_rad", DEFAULT_THUMB_ROLL_OPEN_RAD)
        self.declare_parameter("thumb_yaw_command_gain", -180.0)
        self.declare_parameter("thumb_roll_command_gain", -160.0)
        self.declare_parameter("thumb_yaw_max_delta", 80)
        self.declare_parameter("thumb_roll_max_delta", 80)
        self.declare_parameter("landmark_transform", "pico_native_to_rh")
        self.declare_parameter("wrist_mode", "estimate")
        self.declare_parameter("distal_mode", "dip")

        self._lock = threading.Lock()
        self._latest_msg: ManusGlove | None = None
        self._last_msg_time: float | None = None
        self._last_command: list[int] | None = None
        self._last_thumb_calibrated_command: dict[int, int] | None = None
        self._last_thumb_ik_debug_time = 0.0
        self._thumb_segment_debug: dict[str, Any] | None = None
        self._estop = False

        self._max_delta = int(self.get_parameter("max_delta_per_cycle").value)
        self._alpha = float(self.get_parameter("lowpass_alpha").value)
        self._watchdog_timeout = float(self.get_parameter("watchdog_timeout_sec").value)
        self._reserved_command = clamp_u8(self.get_parameter("reserved_command").value)
        self._mapping_mode = str(self.get_parameter("mapping_mode").value)
        self._neutral_command = _command_parameter(
            self.get_parameter("neutral_command").value,
            STANDARD_OPEN_COMMAND,
        )
        self._closed_command = _command_parameter(
            self.get_parameter("closed_command").value,
            STANDARD_FIST_COMMAND,
        )
        self._lock_neutral_slots = _index_list_parameter(
            self.get_parameter("lock_neutral_slots").value,
            [5, 6, 7, 8, 9, 10, 11, 12, 13, 14],
        )
        if self._mapping_mode == "somehand_ik":
            self._lock_neutral_slots = [index for index in self._lock_neutral_slots if 11 <= index <= 14]
        self._enable_finger_yaw = bool(self.get_parameter("enable_finger_yaw").value)
        self._enable_finger_yaw_mapping = bool(self.get_parameter("enable_finger_yaw_mapping").value)
        self._finger_yaw_mapping: dict[str, Any] | None = None
        if self._enable_finger_yaw or self._enable_finger_yaw_mapping:
            self._lock_neutral_slots = [index for index in self._lock_neutral_slots if index not in range(6, 10)]
        self._enable_thumb_yaw = bool(self.get_parameter("enable_thumb_yaw").value)
        self._enable_thumb_roll = bool(self.get_parameter("enable_thumb_roll").value)
        self._enable_thumb_flexion_mapping = bool(self.get_parameter("enable_thumb_flexion_mapping").value)
        self._thumb_flexion_mapping: dict[str, Any] | None = None
        self._thumb_flexion_root_gamma = max(
            0.05,
            float(self.get_parameter("thumb_flexion_root_gamma").value),
        )
        self._thumb_flexion_tip_gamma = max(
            0.05,
            float(self.get_parameter("thumb_flexion_tip_gamma").value),
        )
        self._enable_thumb_ik = bool(self.get_parameter("enable_thumb_ik").value)
        self._thumb_ik_mode = str(self.get_parameter("thumb_ik_mode").value).strip().lower()
        if self._thumb_ik_mode != "segment":
            self.get_logger().warning(
                f"thumb_ik_mode={self._thumb_ik_mode!r} is deprecated; using 'segment'"
            )
            self._thumb_ik_mode = "segment"
        self._thumb_ik_debug = bool(self.get_parameter("thumb_ik_debug").value)
        self._thumb_segment_start = _landmark_index_parameter(self.get_parameter("thumb_segment_start").value, 2)
        self._thumb_segment_end = _landmark_index_parameter(self.get_parameter("thumb_segment_end").value, 3)
        self._thumb_segment_map_mode = str(self.get_parameter("thumb_segment_map_mode").value).strip().lower()
        if self._thumb_segment_map_mode != "raw":
            self.get_logger().warning(
                f"thumb_segment_map_mode={self._thumb_segment_map_mode!r} is deprecated; using 'raw'"
            )
            self._thumb_segment_map_mode = "raw"
        self._thumb_segment_align_open = bool(self.get_parameter("thumb_segment_align_open").value)
        self._thumb_segment_open_calibration_sec = max(
            0.0,
            float(self.get_parameter("thumb_segment_open_calibration_sec").value),
        )
        self._thumb_segment_manus_open_vector = self._thumb_segment_open_vector_parameter(
            self.get_parameter("thumb_segment_manus_open_vector").value,
            str(self.get_parameter("thumb_segment_manus_open_vector_path").value),
        )
        self._thumb_segment_scale = max(0.01, float(self.get_parameter("thumb_segment_scale").value))
        self._thumb_segment_damping = max(1e-8, float(self.get_parameter("thumb_segment_damping").value))
        self._thumb_segment_max_step = max(1e-4, float(self.get_parameter("thumb_segment_max_step").value))
        self._thumb_robot_segment_body = str(self.get_parameter("thumb_robot_segment_body").value)
        self._thumb_segment_robot_open_command = _command_parameter(
            self.get_parameter("thumb_segment_robot_open_command").value,
            self._neutral_command,
        )
        self._thumb_output_smoothing = bool(self.get_parameter("thumb_output_smoothing").value)
        self._thumb_segment_roll_command_scale = max(
            0.0,
            float(self.get_parameter("thumb_segment_roll_command_scale").value),
        )
        self._thumb_segment_yaw_command_scale = max(
            0.0,
            float(self.get_parameter("thumb_segment_yaw_command_scale").value),
        )
        self._thumb_segment_roll_command_deadzone = max(
            0,
            int(self.get_parameter("thumb_segment_roll_command_deadzone").value),
        )
        self._thumb_segment_roll_command_gamma = max(
            0.05,
            float(self.get_parameter("thumb_segment_roll_command_gamma").value),
        )
        self._thumb_segment_open_samples: list[np.ndarray] = []
        self._thumb_segment_open_start_time: float | None = None
        self._thumb_segment_open_rotation: np.ndarray | None = None
        if self._enable_thumb_yaw:
            self._lock_neutral_slots = [index for index in self._lock_neutral_slots if index != 10]
        if self._enable_thumb_roll:
            self._lock_neutral_slots = [index for index in self._lock_neutral_slots if index != 5]
        if self._enable_thumb_flexion_mapping:
            self._lock_neutral_slots = [index for index in self._lock_neutral_slots if index not in (0, 15)]
        if self._enable_thumb_ik:
            self._lock_neutral_slots = [index for index in self._lock_neutral_slots if index not in (5, 10)]
        self._root_open_rad = _float_list_parameter(
            self.get_parameter("root_flexion_open_rad").value,
            DEFAULT_ROOT_OPEN_RAD,
            length=5,
        )
        self._root_closed_rad = _float_list_parameter(
            self.get_parameter("root_flexion_closed_rad").value,
            DEFAULT_ROOT_CLOSED_RAD,
            length=5,
        )
        self._tip_open_rad = _float_list_parameter(
            self.get_parameter("tip_flexion_open_rad").value,
            DEFAULT_TIP_OPEN_RAD,
            length=5,
        )
        self._tip_closed_rad = _float_list_parameter(
            self.get_parameter("tip_flexion_closed_rad").value,
            DEFAULT_TIP_CLOSED_RAD,
            length=5,
        )
        self._apply_flexion_calibration_path(str(self.get_parameter("flexion_calibration_path").value))
        self._apply_thumb_flexion_mapping_path(str(self.get_parameter("thumb_flexion_mapping_path").value))
        self._finger_yaw_open_rad = _float_list_parameter(
            self.get_parameter("finger_yaw_open_rad").value,
            DEFAULT_FINGER_YAW_OPEN_RAD,
            length=4,
        )
        self._finger_yaw_source = str(self.get_parameter("finger_yaw_source").value)
        self._finger_yaw_command_gain = float(self.get_parameter("finger_yaw_command_gain").value)
        self._finger_yaw_max_delta = max(0, int(self.get_parameter("finger_yaw_max_delta").value))
        self._apply_finger_yaw_calibration_path(str(self.get_parameter("finger_yaw_calibration_path").value))
        self._thumb_yaw_open_rad = float(self.get_parameter("thumb_yaw_open_rad").value)
        self._thumb_roll_open_rad = float(self.get_parameter("thumb_roll_open_rad").value)
        self._thumb_yaw_command_gain = float(self.get_parameter("thumb_yaw_command_gain").value)
        self._thumb_roll_command_gain = float(self.get_parameter("thumb_roll_command_gain").value)
        self._thumb_yaw_max_delta = max(0, int(self.get_parameter("thumb_yaw_max_delta").value))
        self._thumb_roll_max_delta = max(0, int(self.get_parameter("thumb_roll_max_delta").value))
        self._root_gamma = max(0.05, float(self.get_parameter("root_gamma").value))
        self._tip_gamma = max(0.05, float(self.get_parameter("tip_gamma").value))
        if bool(self.get_parameter("start_from_open").value):
            self._last_command = list(self._neutral_command)

        self._hand_frame_cls = None
        self._engine = None
        self._adapter = None
        self._preprocess_landmarks = None
        self._thumb_local_ik = None
        self._thumb_segment_ik = None
        if self._mapping_mode == "somehand_ik" or self._enable_thumb_ik:
            self._load_somehand()
        self._command_pub = self.create_publisher(JointState, self.get_parameter("command_topic").value, 10)
        self.create_subscription(ManusGlove, self.get_parameter("input_topic").value, self._on_glove, 1)
        self.create_subscription(Bool, "/l20/estop", self._on_estop, 1)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._on_timer)
        self.get_logger().info(
            f"MANUS -> LinkerHand retarget node started, "
            f"mapping_mode={self._mapping_mode}, enable_finger_yaw={self._enable_finger_yaw}, "
            f"enable_thumb_yaw={self._enable_thumb_yaw}, enable_thumb_roll={self._enable_thumb_roll}, "
            f"enable_thumb_ik={self._enable_thumb_ik}, "
            f"thumb_ik_mode={self._thumb_ik_mode}"
        )

    def _apply_flexion_calibration_path(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            return
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            self.get_logger().warning(f"flexion_calibration_path not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load flexion calibration {path}: {exc}")
            return
        self._root_open_rad = _float_list_parameter(
            data.get("root_flexion_open_rad"),
            self._root_open_rad,
            length=5,
        )
        self._root_closed_rad = _float_list_parameter(
            data.get("root_flexion_closed_rad"),
            self._root_closed_rad,
            length=5,
        )
        self._tip_open_rad = _float_list_parameter(
            data.get("tip_flexion_open_rad"),
            self._tip_open_rad,
            length=5,
        )
        self._tip_closed_rad = _float_list_parameter(
            data.get("tip_flexion_closed_rad"),
            self._tip_closed_rad,
            length=5,
        )
        self._apply_flexion_command_calibration(data.get("command"))
        self.get_logger().info(
            "loaded flexion calibration "
            f"path={path}, root_open={np.round(self._root_open_rad, 5).tolist()}, "
            f"root_closed={np.round(self._root_closed_rad, 5).tolist()}, "
            f"tip_open={np.round(self._tip_open_rad, 5).tolist()}, "
            f"tip_closed={np.round(self._tip_closed_rad, 5).tolist()}"
        )

    def _apply_flexion_command_calibration(self, command_data: Any) -> None:
        if not isinstance(command_data, dict):
            return
        open_command = _command_parameter(command_data.get("open_command"), self._neutral_command)
        four_closed = _command_parameter(
            command_data.get("four_finger_closed_command"),
            self._closed_command,
        )

        for slot in (1, 2, 3, 4, 16, 17, 18, 19):
            self._neutral_command[slot] = open_command[slot]
        for slot in (1, 2, 3, 4, 16, 17, 18, 19):
            self._closed_command[slot] = four_closed[slot]

    def _apply_finger_yaw_calibration_path(self, path_value: str) -> None:
        if not self._enable_finger_yaw_mapping:
            return
        path_text = str(path_value).strip()
        if not path_text:
            self.get_logger().warning("enable_finger_yaw_mapping=true but finger_yaw_calibration_path is empty")
            return
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            self.get_logger().warning(f"finger_yaw_calibration_path not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load finger yaw calibration {path}: {exc}")
            return

        samples = data.get("samples")
        command_data = data.get("command")
        if not isinstance(samples, dict) or not isinstance(command_data, dict):
            self.get_logger().warning(f"invalid finger yaw calibration file: {path}")
            return
        open_sample = samples.get("natural_open")
        close_sample = samples.get("finger_close")
        spread_sample = samples.get("finger_spread")
        if not all(isinstance(sample, dict) for sample in (open_sample, close_sample, spread_sample)):
            self.get_logger().warning(f"finger yaw calibration missing required samples: {path}")
            return

        open_command = _command_parameter(command_data.get("natural_open_command"), self._neutral_command)
        close_command = _command_parameter(command_data.get("finger_close_command"), self._neutral_command)
        spread_command = _command_parameter(command_data.get("finger_spread_command"), self._neutral_command)
        source = str(data.get("source", self._finger_yaw_source)).strip().lower()
        if source not in (
            "pip",
            "dip",
            "tip",
            "mcp_orientation",
            "pip_orientation",
            "ip_orientation",
            "dip_orientation",
        ):
            self.get_logger().warning(f"unknown finger yaw calibration source={source!r}; using {self._finger_yaw_source!r}")
            source = self._finger_yaw_source
        try:
            self._finger_yaw_mapping = {
                "source": source,
                "open_rad": _float_list_parameter(open_sample.get("yaw_rad"), DEFAULT_FINGER_YAW_OPEN_RAD, length=4),
                "close_rad": _float_list_parameter(close_sample.get("yaw_rad"), DEFAULT_FINGER_YAW_OPEN_RAD, length=4),
                "spread_rad": _float_list_parameter(spread_sample.get("yaw_rad"), DEFAULT_FINGER_YAW_OPEN_RAD, length=4),
                "open_cmd": [open_command[slot] for slot in range(6, 10)],
                "close_cmd": [close_command[slot] for slot in range(6, 10)],
                "spread_cmd": [spread_command[slot] for slot in range(6, 10)],
            }
        except (TypeError, ValueError, IndexError) as exc:
            self.get_logger().warning(f"invalid finger yaw calibration values in {path}: {exc}")
            self._finger_yaw_mapping = None
            return
        if not any(
            abs(float(self._finger_yaw_mapping[target][index]) - float(self._finger_yaw_mapping["open_rad"][index])) > 1e-5
            for target in ("close_rad", "spread_rad")
            for index in range(4)
        ):
            self.get_logger().warning(
                "finger yaw calibration has no effective yaw range; "
                "close/spread MANUS yaw samples are equal to natural_open"
            )
        self.get_logger().info(
            "loaded finger yaw calibration "
            f"path={path}, source={source}, "
            f"open_rad={np.round(self._finger_yaw_mapping['open_rad'], 5).tolist()}, "
            f"close_rad={np.round(self._finger_yaw_mapping['close_rad'], 5).tolist()}, "
            f"spread_rad={np.round(self._finger_yaw_mapping['spread_rad'], 5).tolist()}, "
            f"open_cmd={self._finger_yaw_mapping['open_cmd']}, "
            f"close_cmd={self._finger_yaw_mapping['close_cmd']}, "
            f"spread_cmd={self._finger_yaw_mapping['spread_cmd']}"
        )

    def _apply_thumb_flexion_mapping_path(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            return
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            self.get_logger().warning(f"thumb_flexion_mapping_path not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load thumb flexion mapping {path}: {exc}")
            return

        samples = data.get("samples")
        command_data = data.get("command")
        if not isinstance(samples, dict) or not isinstance(command_data, dict):
            self.get_logger().warning(f"invalid thumb flexion mapping file: {path}")
            return
        open_sample = samples.get("thumb_natural_open")
        touch_sample = samples.get("thumb_pinky_root_touch")
        open_command = _command_parameter(command_data.get("thumb_natural_open_command"), self._neutral_command)
        touch_command = _command_parameter(command_data.get("thumb_pinky_root_touch_command"), self._neutral_command)
        if not isinstance(open_sample, dict) or not isinstance(touch_sample, dict):
            self.get_logger().warning(f"thumb flexion mapping missing required samples: {path}")
            return
        try:
            self._thumb_flexion_mapping = {
                "root_open_rad": float(open_sample["root_rad"]),
                "root_touch_rad": float(touch_sample["root_rad"]),
                "tip_open_rad": float(open_sample["tip_rad"]),
                "tip_touch_rad": float(touch_sample["tip_rad"]),
                "root_open_cmd": int(open_command[0]),
                "root_touch_cmd": int(touch_command[0]),
                "tip_open_cmd": int(open_command[15]),
                "tip_touch_cmd": int(touch_command[15]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid thumb flexion mapping values in {path}: {exc}")
            self._thumb_flexion_mapping = None
            return
        self.get_logger().info(
            "loaded thumb flexion mapping "
            f"path={path}, root_rad=[{self._thumb_flexion_mapping['root_open_rad']:.5f}, "
            f"{self._thumb_flexion_mapping['root_touch_rad']:.5f}], "
            f"tip_rad=[{self._thumb_flexion_mapping['tip_open_rad']:.5f}, "
            f"{self._thumb_flexion_mapping['tip_touch_rad']:.5f}], "
            f"root_cmd=[{self._thumb_flexion_mapping['root_open_cmd']}, "
            f"{self._thumb_flexion_mapping['root_touch_cmd']}], "
            f"tip_cmd=[{self._thumb_flexion_mapping['tip_open_cmd']}, "
            f"{self._thumb_flexion_mapping['tip_touch_cmd']}]"
        )

    def _load_somehand(self) -> None:
        somehand_root = Path(str(self.get_parameter("somehand_root").value)).expanduser().resolve()
        src_path = somehand_root / "src"
        if str(src_path) not in sys.path:
            sys.path.insert(0, str(src_path))

        from somehand.api import HandFrame, RetargetingEngine
        from somehand.domain import preprocess_landmarks
        from somehand.infrastructure.controllers.adapters import LinkerHandModelAdapter

        config_path = Path(str(self.get_parameter("somehand_config_path").value)).expanduser().resolve()
        sdk_root = Path(str(self.get_parameter("linkerhand_sdk_root").value)).expanduser().resolve()
        family = str(self.get_parameter("hand_family").value).upper()

        self._hand_frame_cls = HandFrame
        self._preprocess_landmarks = preprocess_landmarks
        self._engine = RetargetingEngine.from_config_path(str(config_path), input_type="landmarks")
        self._adapter = LinkerHandModelAdapter(
            self._engine.hand_model,
            family=family,
            hand_side=self._engine.config.hand.side,
            sdk_root=str(sdk_root),
        )
        if self._enable_thumb_ik:
            self._thumb_local_ik = _ThumbLocalIK(self._engine.hand_model)
            self._thumb_segment_ik = _ThumbSegmentIK(
                self._engine.hand_model,
                adapter=self._adapter,
                thumb_ik=self._thumb_local_ik,
                robot_segment_body=self._thumb_robot_segment_body,
                robot_open_command=self._thumb_segment_robot_open_command,
            )
        self.get_logger().info(
            f"loaded somehand config={config_path}, family={family}, sdk_root={sdk_root}"
        )

    def _on_glove(self, msg: ManusGlove) -> None:
        with self._lock:
            self._latest_msg = msg
            self._last_msg_time = monotonic()

    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)
        if self._estop:
            self._last_command = list(self._neutral_command)
            self._publish(self._last_command)

    def _on_timer(self) -> None:
        with self._lock:
            msg = self._latest_msg
            last_msg_time = self._last_msg_time

        if self._estop:
            self._publish(self._neutral_command)
            return

        if msg is None:
            return
        if last_msg_time is not None and monotonic() - last_msg_time > self._watchdog_timeout:
            self._last_command = self._filter_command(self._neutral_command)
            self._publish(self._last_command)
            return

        try:
            command = self._command_from_manus(msg)
        except Exception as exc:
            self.get_logger().warning(f"failed to retarget MANUS frame through somehand: {exc}")
            return

        filtered_command = self._filter_command(command)
        self._debug_thumb_segment_publish(command, filtered_command)
        self._last_command = filtered_command
        self._publish(self._last_command)

    def _thumb_segment_open_vector_parameter(self, inline_value: Any, path_value: str) -> np.ndarray | None:
        inline_vector = _optional_vector3_parameter(inline_value)
        if inline_vector is not None:
            self.get_logger().info(
                "using inline thumb segment MANUS open vector "
                f"{np.round(inline_vector, 6).tolist()}"
            )
            return inline_vector

        path_text = str(path_value).strip()
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        if not path.exists():
            self.get_logger().info(
                f"thumb segment MANUS open vector file not found: {path}; "
                "falling back to startup open calibration"
            )
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            vector = _optional_vector3_parameter(data.get("manus_open_vector"))
            if vector is None:
                raise ValueError("missing manus_open_vector")
            segment_start = data.get("segment_start")
            segment_end = data.get("segment_end")
            if segment_start is not None and int(segment_start) != self._thumb_segment_start:
                self.get_logger().warning(
                    f"thumb segment open vector start={segment_start} does not match "
                    f"thumb_segment_start={self._thumb_segment_start}"
                )
            if segment_end is not None and int(segment_end) != self._thumb_segment_end:
                self.get_logger().warning(
                    f"thumb segment open vector end={segment_end} does not match "
                    f"thumb_segment_end={self._thumb_segment_end}"
                )
            self.get_logger().info(
                f"loaded thumb segment MANUS open vector path={path}, "
                f"vector={np.round(vector, 6).tolist()}"
            )
            return vector
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(
                f"failed to load thumb segment MANUS open vector from {path}: {exc}; "
                "falling back to startup open calibration"
            )
            return None

    def _command_from_manus(self, msg: ManusGlove) -> list[int]:
        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=str(self.get_parameter("landmark_transform").value),
            wrist_mode=str(self.get_parameter("wrist_mode").value),
            distal_mode=str(self.get_parameter("distal_mode").value),
        )
        hand_side = "right" if str(msg.side).lower() != "left" else "left"
        if self._mapping_mode != "somehand_ik":
            return self._command_from_landmark_flexion(landmarks, hand_side, msg.raw_nodes)
        command = self._somehand_ik_command(landmarks, hand_side)
        self._apply_neutral_locks(command)
        return command

    def _somehand_ik_command(self, landmarks: np.ndarray, hand_side: str) -> list[int]:
        if self._hand_frame_cls is None or self._engine is None or self._adapter is None:
            raise RuntimeError("somehand IK requested but somehand was not loaded")
        frame = self._hand_frame_cls(
            landmarks_3d=landmarks,
            landmarks_2d=np.zeros((21, 2), dtype=np.float64),
            hand_side=hand_side,
        )
        result = self._engine.process(frame)
        command = [clamp_u8(value) for value in self._adapter.qpos_to_sdk_range(result.qpos)]
        if len(command) != 20:
            raise ValueError(f"expected 20 LinkerHand command values, got {len(command)}")
        for index in range(11, 15):
            command[index] = self._reserved_command
        return command

    def _command_from_landmark_flexion(
        self,
        landmarks: np.ndarray,
        hand_side: str,
        raw_nodes: list[Any] | None = None,
    ) -> list[int]:
        command = list(self._neutral_command)
        for finger_index, (mcp, pip, dip, tip) in enumerate(FINGER_LANDMARKS):
            if finger_index == 0:
                continue
            root_angle = _joint_flexion_rad(landmarks[mcp], landmarks[pip], landmarks[dip])
            tip_angle = _joint_flexion_rad(landmarks[pip], landmarks[dip], landmarks[tip])
            root_amount = _normalized_angle(
                root_angle,
                self._root_open_rad[finger_index],
                self._root_closed_rad[finger_index],
                self._root_gamma,
            )
            tip_amount = _normalized_angle(
                tip_angle,
                self._tip_open_rad[finger_index],
                self._tip_closed_rad[finger_index],
                self._tip_gamma,
            )
            command[finger_index] = _lerp_command(
                self._neutral_command[finger_index],
                self._closed_command[finger_index],
                root_amount,
            )
            command[15 + finger_index] = _lerp_command(
                self._neutral_command[15 + finger_index],
                self._closed_command[15 + finger_index],
                tip_amount,
            )
        if self._enable_finger_yaw:
            self._apply_finger_yaw(command, landmarks, raw_nodes)
        if self._enable_thumb_flexion_mapping:
            self._apply_thumb_flexion_mapping(command, landmarks)
        if self._enable_thumb_yaw or self._enable_thumb_roll:
            self._apply_thumb_pose(command, landmarks)
        if self._enable_thumb_ik:
            self._apply_thumb_ik(command, landmarks, hand_side)
        for index in range(11, 15):
            command[index] = self._reserved_command
        self._apply_neutral_locks(command)
        return command

    def _apply_finger_yaw(
        self,
        command: list[int],
        landmarks: np.ndarray,
        raw_nodes: list[Any] | None = None,
    ) -> None:
        if self._finger_yaw_mapping is not None:
            source = str(self._finger_yaw_mapping["source"])
            if source == "mcp_orientation":
                if raw_nodes is None:
                    return
                yaw_angles = _finger_mcp_orientation_yaw_rad(raw_nodes)
            elif source in ("pip_orientation", "ip_orientation", "dip_orientation"):
                if raw_nodes is None:
                    return
                joint_type = {
                    "pip_orientation": "PIP",
                    "ip_orientation": "IP",
                    "dip_orientation": "DIP",
                }[source]
                yaw_angles = _finger_joint_orientation_yaw_rad(raw_nodes, joint_type=joint_type)
            else:
                yaw_angles = _finger_yaw_rad(landmarks, source=source)
            open_rad = self._finger_yaw_mapping["open_rad"]
            close_rad = self._finger_yaw_mapping["close_rad"]
            spread_rad = self._finger_yaw_mapping["spread_rad"]
            open_cmd = self._finger_yaw_mapping["open_cmd"]
            close_cmd = self._finger_yaw_mapping["close_cmd"]
            spread_cmd = self._finger_yaw_mapping["spread_cmd"]
            for local_index, angle in enumerate(yaw_angles):
                slot = 6 + local_index
                close_amount = _normalized_signed_segment(
                    float(angle),
                    float(open_rad[local_index]),
                    float(close_rad[local_index]),
                )
                spread_amount = _normalized_signed_segment(
                    float(angle),
                    float(open_rad[local_index]),
                    float(spread_rad[local_index]),
                )
                if close_amount >= spread_amount:
                    command[slot] = _lerp_command(
                        int(open_cmd[local_index]),
                        int(close_cmd[local_index]),
                        close_amount,
                    )
                else:
                    command[slot] = _lerp_command(
                        int(open_cmd[local_index]),
                        int(spread_cmd[local_index]),
                        spread_amount,
                    )
            return

        yaw_angles = _finger_yaw_rad(landmarks, source=self._finger_yaw_source)
        for local_index, angle in enumerate(yaw_angles):
            slot = 6 + local_index
            neutral = self._neutral_command[slot]
            delta = self._finger_yaw_command_gain * (float(angle) - self._finger_yaw_open_rad[local_index])
            delta = max(-self._finger_yaw_max_delta, min(self._finger_yaw_max_delta, delta))
            command[slot] = clamp_u8(neutral + delta)

    def _apply_thumb_pose(self, command: list[int], landmarks: np.ndarray) -> None:
        thumb_pose = _thumb_pose_features(landmarks)
        if self._enable_thumb_roll:
            neutral = self._neutral_command[5]
            delta = self._thumb_roll_command_gain * (thumb_pose["roll_rad"] - self._thumb_roll_open_rad)
            delta = max(-self._thumb_roll_max_delta, min(self._thumb_roll_max_delta, delta))
            command[5] = clamp_u8(neutral + delta)
        if self._enable_thumb_yaw:
            neutral = self._neutral_command[10]
            delta = self._thumb_yaw_command_gain * (thumb_pose["yaw_rad"] - self._thumb_yaw_open_rad)
            delta = max(-self._thumb_yaw_max_delta, min(self._thumb_yaw_max_delta, delta))
            command[10] = clamp_u8(neutral + delta)

    def _apply_thumb_flexion_mapping(self, command: list[int], landmarks: np.ndarray) -> None:
        mapping = self._thumb_flexion_mapping
        if mapping is None:
            return
        root_angle = _joint_flexion_rad(landmarks[1], landmarks[2], landmarks[3])
        tip_angle = _joint_flexion_rad(landmarks[2], landmarks[3], landmarks[4])
        root_amount = _normalized_angle(
            root_angle,
            mapping["root_open_rad"],
            mapping["root_touch_rad"],
            self._thumb_flexion_root_gamma,
        )
        tip_amount = _normalized_angle(
            tip_angle,
            mapping["tip_open_rad"],
            mapping["tip_touch_rad"],
            self._thumb_flexion_tip_gamma,
        )
        command[0] = _lerp_command(mapping["root_open_cmd"], mapping["root_touch_cmd"], root_amount)
        command[15] = _lerp_command(mapping["tip_open_cmd"], mapping["tip_touch_cmd"], tip_amount)

    def _apply_thumb_ik(self, command: list[int], landmarks: np.ndarray, hand_side: str) -> None:
        ik_command = self._thumb_segment_ik_command(command, landmarks)
        values = {slot: ik_command[slot] for slot in THUMB_IK_COMMAND_SLOTS}
        raw_values = dict(values)
        if self._thumb_output_smoothing:
            values = self._smooth_thumb_command(values)
        if self._thumb_segment_debug is not None:
            self._thumb_segment_debug["raw_cmd"] = [raw_values[index] for index in THUMB_IK_COMMAND_SLOTS]
            self._thumb_segment_debug["smooth_cmd"] = [values[index] for index in THUMB_IK_COMMAND_SLOTS]
        for slot, value in values.items():
            command[slot] = value

    def _thumb_segment_ik_command(self, base_command: list[int], landmarks: np.ndarray) -> list[int]:
        if self._adapter is None or self._thumb_segment_ik is None:
            raise RuntimeError("thumb segment IK requested but somehand was not loaded")
        base_qpos = self._adapter.sdk_range_to_qpos(base_command)
        target_vector = self._thumb_segment_target_vector(landmarks)
        result_qpos = self._thumb_segment_ik.solve_target(
            target_vector,
            base_qpos,
            damping=self._thumb_segment_damping,
            max_step=self._thumb_segment_max_step,
        )
        command = [clamp_u8(value) for value in self._adapter.qpos_to_sdk_range(result_qpos)]
        ik_roll_yaw_command = [command[5], command[10]]
        command[5] = _scale_command_delta_ease_in_with_deadzone(
            command[5],
            self._thumb_segment_robot_open_command[5],
            self._thumb_segment_roll_command_scale,
            self._thumb_segment_roll_command_gamma,
            self._thumb_segment_roll_command_deadzone,
        )
        command[10] = _scale_command_delta(
            command[10],
            self._thumb_segment_robot_open_command[10],
            self._thumb_segment_yaw_command_scale,
        )
        self._thumb_segment_debug = {
            "start": self._thumb_segment_start,
            "end": self._thumb_segment_end,
            "map": self._thumb_segment_map_mode,
            "align": self._thumb_segment_align_status(),
            "target": np.round(target_vector, 5).tolist(),
            "actual": np.round(self._thumb_segment_ik.current_segment_vector(), 5).tolist(),
            "residual": float(np.linalg.norm(self._thumb_segment_ik.current_segment_vector() - target_vector)),
            "roll_yaw_open": [
                int(self._thumb_segment_robot_open_command[5]),
                int(self._thumb_segment_robot_open_command[10]),
            ],
            "roll_yaw_scale": [
                float(self._thumb_segment_roll_command_scale),
                float(self._thumb_segment_yaw_command_scale),
            ],
            "roll_deadzone_gamma": [
                int(self._thumb_segment_roll_command_deadzone),
                float(self._thumb_segment_roll_command_gamma),
            ],
            "roll_yaw_ik": ik_roll_yaw_command,
            "base_cmd": [base_command[index] for index in THUMB_COMMAND_SLOTS],
            "ik_cmd": [command[index] for index in THUMB_COMMAND_SLOTS],
        }
        for index in range(11, 15):
            command[index] = self._reserved_command
        return command

    def _debug_thumb_segment_publish(self, raw_command: list[int], filtered_command: list[int]) -> None:
        if not self._thumb_ik_debug or self._thumb_ik_mode != "segment":
            return
        if self._thumb_segment_debug is None:
            return
        now = monotonic()
        if now - self._last_thumb_ik_debug_time < 0.5:
            return
        self._last_thumb_ik_debug_time = now
        debug = self._thumb_segment_debug
        self.get_logger().info(
            "thumb_segment_debug "
            f"start={debug['start']} end={debug['end']} "
            f"map={debug['map']} align={debug['align']} "
            f"target={debug['target']} actual={debug['actual']} "
            f"residual={debug['residual']:.5f} "
            f"roll_yaw_open={debug['roll_yaw_open']} "
            f"roll_yaw_scale={debug['roll_yaw_scale']} "
            f"roll_dz_gamma={debug['roll_deadzone_gamma']} "
            f"roll_yaw_ik={debug['roll_yaw_ik']} "
            f"base_cmd={debug['base_cmd']} ik_cmd={debug['ik_cmd']} "
            f"raw_cmd={[raw_command[index] for index in THUMB_IK_COMMAND_SLOTS]} "
            f"smooth_cmd={debug.get('smooth_cmd')} "
            f"pub_cmd={[filtered_command[index] for index in THUMB_IK_COMMAND_SLOTS]}"
        )

    def _thumb_segment_target_vector(self, landmarks: np.ndarray) -> np.ndarray:
        if self._thumb_segment_ik is None:
            raise RuntimeError("thumb segment IK target requested before initialization")
        start = _manus_thumb_local_point_at(landmarks, self._thumb_segment_start)
        end = _manus_thumb_local_point_at(landmarks, self._thumb_segment_end)
        vector = end if self._thumb_segment_start == self._thumb_segment_end else end - start
        vector = self._align_thumb_segment_open(vector)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-8:
            return self._thumb_segment_ik.robot_open_segment_vector.copy()
        return vector / norm * self._thumb_segment_ik.segment_length * self._thumb_segment_scale

    def _align_thumb_segment_open(self, vector: np.ndarray) -> np.ndarray:
        vector = np.asarray(vector, dtype=np.float64)
        if not self._thumb_segment_align_open or self._thumb_segment_ik is None:
            return vector
        unit = _unit_vector(vector)
        if unit is None:
            return vector
        if self._thumb_segment_open_rotation is not None:
            return self._thumb_segment_open_rotation @ vector

        if self._thumb_segment_manus_open_vector is not None:
            open_unit = _unit_vector(self._thumb_segment_manus_open_vector)
            robot_open_unit = _unit_vector(self._thumb_segment_ik.robot_open_segment_vector)
            if open_unit is None or robot_open_unit is None:
                self._thumb_segment_open_rotation = np.eye(3, dtype=np.float64)
            else:
                self._thumb_segment_open_rotation = _rotation_between(open_unit, robot_open_unit)
            return self._thumb_segment_open_rotation @ vector

        now = monotonic()
        if self._thumb_segment_open_start_time is None:
            self._thumb_segment_open_start_time = now
        self._thumb_segment_open_samples.append(unit)
        if now - self._thumb_segment_open_start_time < self._thumb_segment_open_calibration_sec:
            return self._thumb_segment_ik.robot_open_segment_vector.copy()

        open_unit = _unit_vector(np.mean(np.asarray(self._thumb_segment_open_samples, dtype=np.float64), axis=0))
        robot_open_unit = _unit_vector(self._thumb_segment_ik.robot_open_segment_vector)
        if open_unit is None or robot_open_unit is None:
            self._thumb_segment_open_rotation = np.eye(3, dtype=np.float64)
        else:
            self._thumb_segment_open_rotation = _rotation_between(open_unit, robot_open_unit)
        return self._thumb_segment_open_rotation @ vector

    def _thumb_segment_align_status(self) -> str:
        if self._thumb_ik_mode != "segment" or not self._thumb_segment_align_open:
            return "off"
        if self._thumb_segment_manus_open_vector is not None:
            return "fixed" if self._thumb_segment_open_rotation is not None else "fixed_pending"
        return "done" if self._thumb_segment_open_rotation is not None else "calibrating"

    def _smooth_thumb_command(self, values: dict[int, int]) -> dict[int, int]:
        slots = tuple(values.keys())
        current = {slot: clamp_u8(values[slot]) for slot in slots}
        if self._last_thumb_calibrated_command is None:
            self._last_thumb_calibrated_command = dict(current)
            return current

        smoothed: dict[int, int] = {}
        for slot in slots:
            previous = self._last_thumb_calibrated_command.get(slot, current[slot])
            target = current[slot]
            if abs(target - previous) <= THUMB_OUTPUT_DEADBAND:
                smoothed[slot] = previous
                continue
            limited = previous + max(-THUMB_OUTPUT_MAX_DELTA, min(THUMB_OUTPUT_MAX_DELTA, target - previous))
            smoothed[slot] = clamp_u8(previous + THUMB_OUTPUT_ALPHA * (limited - previous))
        self._last_thumb_calibrated_command = dict(smoothed)
        return smoothed

    def _apply_neutral_locks(self, command: list[int]) -> None:
        for index in self._lock_neutral_slots:
            command[index] = self._neutral_command[index]

    def _filter_command(self, command: list[int]) -> list[int]:
        current = [clamp_u8(value) for value in command]
        if self._last_command is None:
            return current

        rate_limited: list[int] = []
        for previous, value in zip(self._last_command, current):
            delta = max(-self._max_delta, min(self._max_delta, value - previous))
            rate_limited.append(previous + delta)

        alpha = max(0.0, min(1.0, self._alpha))
        return [
            clamp_u8(previous + alpha * (value - previous))
            for previous, value in zip(self._last_command, rate_limited)
        ]

    def _publish(self, command: list[int]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "Thumb Base",
            "Index Finger Base",
            "Middle Finger Base",
            "Ring Finger Base",
            "Pinky Finger Base",
            "Thumb Roll",
            "Index Finger Yaw",
            "Middle Finger Yaw",
            "Ring Finger Yaw",
            "Pinky Finger Yaw",
            "Thumb Yaw",
            "Reserved",
            "Reserved",
            "Reserved",
            "Reserved",
            "Thumb Tip",
            "Index Finger Tip",
            "Middle Finger Tip",
            "Ring Finger Tip",
            "Pinky Finger Tip",
        ]
        msg.position = [float(value) for value in command]
        msg.velocity = [0.0] * 20
        msg.effort = [0.0] * 20
        self._command_pub.publish(msg)


def _command_parameter(value: Any, default: list[int]) -> list[int]:
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return list(default)
        try:
            parsed = yaml.safe_load(text)
        except yaml.YAMLError:
            parsed = None
        if isinstance(parsed, (list, tuple)) and len(parsed) == 20:
            return [clamp_u8(item) for item in parsed]
    if isinstance(value, (list, tuple)) and len(value) == 20:
        return [clamp_u8(item) for item in value]
    return list(default)


def _index_list_parameter(value: Any, default: list[int]) -> list[int]:
    if not isinstance(value, (list, tuple)):
        return list(default)
    indexes: list[int] = []
    for item in value:
        index = int(item)
        if 0 <= index < 20 and index not in indexes:
            indexes.append(index)
    return indexes


def _landmark_index_parameter(value: Any, default: int) -> int:
    try:
        index = int(value)
    except (TypeError, ValueError):
        return int(default)
    return max(0, min(20, index))


def _float_list_parameter(value: Any, default: list[float], *, length: int) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) == length:
        return [float(item) for item in value]
    return list(default)


def _optional_vector3_parameter(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    if isinstance(value, str) and not value.strip():
        return None
    sentinel = [float("nan"), float("nan"), float("nan")]
    vector = _vector3_parameter(value, sentinel)
    if np.isnan(vector).any():
        return None
    return vector


def _vector3_parameter(value: Any, default: list[float]) -> np.ndarray:
    if isinstance(value, str):
        text = value.strip()
        if text:
            try:
                parsed = yaml.safe_load(text)
            except yaml.YAMLError:
                parsed = None
            if isinstance(parsed, str):
                parsed = [part.strip() for part in parsed.split(",")]
            elif parsed is None and "," in text:
                parsed = [part.strip() for part in text.split(",")]
            if isinstance(parsed, (list, tuple)) and len(parsed) == 3:
                return np.asarray([float(item) for item in parsed], dtype=np.float64)
            if "," in text:
                parts = [part.strip() for part in text.split(",")]
                if len(parts) == 3:
                    return np.asarray([float(item) for item in parts], dtype=np.float64)
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return np.asarray([float(item) for item in value], dtype=np.float64)
    return np.asarray(default, dtype=np.float64)


def _fit_similarity(source: np.ndarray, target: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    covariance = source_zero.T @ target_zero
    u, singular_values, vh = np.linalg.svd(covariance)
    rotation = u @ vh
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh
    scale = float(np.sum(singular_values) / max(np.sum(source_zero * source_zero), 1e-8))
    translation = target_center - scale * (source_center @ rotation)
    return scale, rotation, translation


def _joint_flexion_rad(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    first = a - b
    second = c - b
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= 1e-8:
        return 0.0
    cosine = float(np.dot(first, second) / denominator)
    interior_angle = math.acos(max(-1.0, min(1.0, cosine)))
    return math.pi - interior_angle


def _normalized_angle(angle: float, open_angle: float, closed_angle: float, gamma: float) -> float:
    span = max(1e-6, float(closed_angle) - float(open_angle))
    amount = max(0.0, min(1.0, (float(angle) - float(open_angle)) / span))
    return amount ** gamma


def _lerp_command(open_value: int, closed_value: int, amount: float) -> int:
    return clamp_u8(float(open_value) + float(amount) * (float(closed_value) - float(open_value)))


def _limit_toward_reference(value: int, reference: int, *, max_delta: int) -> int:
    return clamp_u8(float(reference) + max(-max_delta, min(max_delta, int(value) - int(reference))))


def _scale_command_flexion(value: int, open_value: int, scale: float) -> int:
    return clamp_u8(float(open_value) + float(scale) * (float(value) - float(open_value)))


def _scale_command_delta(value: int, neutral_value: int, scale: float) -> int:
    return clamp_u8(float(neutral_value) + float(scale) * (float(value) - float(neutral_value)))


def _scale_command_delta_ease_in_with_deadzone(
    value: int,
    neutral_value: int,
    scale: float,
    gamma: float,
    deadzone: int,
) -> int:
    delta = float(value) - float(neutral_value)
    deadzone_value = max(0.0, float(deadzone))
    if abs(delta) <= deadzone_value:
        return clamp_u8(neutral_value)
    headroom = float(255 - neutral_value) if delta > 0.0 else float(neutral_value)
    usable = max(1e-6, headroom - deadzone_value)
    amount = min(1.0, (abs(delta) - deadzone_value) / usable)
    eased_delta = float(scale) * usable * (amount ** float(gamma))
    return clamp_u8(float(neutral_value) + math.copysign(eased_delta, delta))


def _scale_command_delta_ease_in(value: int, neutral_value: int, scale: float, gamma: float) -> int:
    delta = float(value) - float(neutral_value)
    if abs(delta) <= 1e-6:
        return clamp_u8(value)
    headroom = float(255 - neutral_value) if delta > 0.0 else float(neutral_value)
    if headroom <= 1e-6:
        return clamp_u8(value)
    amount = min(1.0, abs(delta) / headroom)
    eased = min(1.0, float(scale) * (amount ** float(gamma)))
    return clamp_u8(float(neutral_value) + math.copysign(headroom * eased, delta))


def _scale_command_delta_ease_in_to_reference(
    value: int,
    neutral_value: int,
    reference_value: int,
    scale: float,
    gamma: float,
    overdrive: float,
    deadzone: int,
) -> int:
    delta = float(value) - float(neutral_value)
    reference_delta = float(reference_value) - float(neutral_value)
    deadzone_value = max(0.0, float(deadzone))
    if abs(delta) <= deadzone_value:
        return clamp_u8(neutral_value)
    if abs(reference_delta) <= deadzone_value + 1e-6:
        return _scale_command_delta_ease_in(value, neutral_value, scale, gamma)
    if delta * reference_delta < 0.0:
        return _scale_command_delta_ease_in(value, neutral_value, scale, gamma)
    amount = min(1.0, (abs(delta) - deadzone_value) / max(1e-6, abs(reference_delta) - deadzone_value))
    eased = float(overdrive) * (amount ** float(gamma))
    return clamp_u8(float(neutral_value) + math.copysign(abs(reference_delta) * eased, reference_delta))


class _ThumbLocalIK:
    def __init__(self, hand_model: Any) -> None:
        import mujoco

        self._mujoco = mujoco
        self.hand_model = hand_model
        self.model = hand_model.model
        self.data = hand_model.data
        self.roll_body_id = self._resolve_body("thumb_metacarpals_base2")
        self.tip_site_id = self._resolve_site("thumb_distal_tip")
        active_joint_names = ("thumb_cmc_yaw", "thumb_cmc_roll", "thumb_cmc_pitch", "thumb_mcp")
        joint_index = hand_model.get_joint_name_to_qpos_index()
        self.active_joint_names = tuple(name for name in active_joint_names if name in joint_index)
        self.active_qpos_ids = self._active_qpos_ids(
            self.active_joint_names
        )
        self.yaw_qpos_id = self._active_joint_qpos_id("thumb_cmc_yaw")
        self.active_dof_ids = self._qpos_ids_to_dof_ids(self.active_qpos_ids)
        self.lower, self.upper = self._active_joint_ranges(self.active_qpos_ids)
        self._open_qpos = hand_model.get_qpos()
        self._robot_thumb_chain_length = self._thumb_chain_length()
        self._last_qpos: np.ndarray | None = None

    def solve(self, human_vector: np.ndarray, human_chain_length: float, base_qpos: np.ndarray) -> np.ndarray:
        target = self._target_vector(human_vector, human_chain_length)
        return self.solve_target(target, base_qpos)

    def solve_target(
        self,
        target: np.ndarray,
        base_qpos: np.ndarray,
        seed_qposes: list[np.ndarray] | None = None,
        posture_reference_qpos: np.ndarray | None = None,
        yaw_bias_qpos: float | None = None,
        yaw_bias_weight: float = 0.0,
    ) -> np.ndarray:
        target = np.asarray(target, dtype=np.float64)
        base_qpos = np.asarray(base_qpos, dtype=np.float64).copy()
        posture_reference = (
            np.asarray(posture_reference_qpos, dtype=np.float64).copy()
            if posture_reference_qpos is not None
            else None
        )
        seeds: list[np.ndarray] = []
        if self._last_qpos is not None:
            seeds.append(self._last_qpos.copy())
        if seed_qposes:
            seeds.extend(np.asarray(seed, dtype=np.float64).copy() for seed in seed_qposes)
        if posture_reference is not None:
            seeds.append(posture_reference.copy())
        seeds.append(base_qpos.copy())

        best_qpos: np.ndarray | None = None
        best_score = float("inf")
        for seed in seeds:
            qpos = seed.copy()
            self.hand_model.apply_mimic_constraints(qpos)
            solved = self._solve_target_from_seed(
                target,
                qpos,
                posture_reference if posture_reference is not None else seed,
                yaw_bias_qpos=yaw_bias_qpos,
                yaw_bias_weight=yaw_bias_weight,
            )
            self.hand_model.set_qpos(solved)
            residual = float(np.linalg.norm(self._tip_vector() - target))
            if self._last_qpos is not None:
                motion = float(np.linalg.norm(solved[self.active_qpos_ids] - self._last_qpos[self.active_qpos_ids]))
            else:
                motion = 0.0
            posture_error = 0.0
            if posture_reference is not None:
                posture_error = float(
                    np.linalg.norm(
                        (solved[self.active_qpos_ids] - posture_reference[self.active_qpos_ids])
                        * self._posture_score_weights()
                    )
                )
            yaw_error = 0.0
            if yaw_bias_qpos is not None and self.yaw_qpos_id is not None:
                yaw_error = abs(float(solved[self.yaw_qpos_id]) - float(yaw_bias_qpos))
            score = residual + 2e-3 * motion + 4e-3 * posture_error + 2e-3 * float(yaw_bias_weight) * yaw_error
            if score < best_score:
                best_score = score
                best_qpos = solved.copy()

        if best_qpos is None:
            best_qpos = base_qpos.copy()
        self.hand_model.set_qpos(best_qpos)
        self._last_qpos = best_qpos.copy()
        return best_qpos

    def _solve_target_from_seed(
        self,
        target: np.ndarray,
        qpos: np.ndarray,
        reference: np.ndarray,
        *,
        yaw_bias_qpos: float | None = None,
        yaw_bias_weight: float = 0.0,
    ) -> np.ndarray:
        posture_weights = self._posture_solver_weights()
        for _ in range(32):
            self.hand_model.set_qpos(qpos)
            residual = self._tip_vector() - target
            if float(np.linalg.norm(residual)) <= 8e-4:
                break
            jacobian = self._tip_vector_jacobian()
            active = qpos[self.active_qpos_ids]
            reference_active = reference[self.active_qpos_ids]
            posture_weights_active = posture_weights.copy()
            if yaw_bias_qpos is not None and "thumb_cmc_yaw" in self.active_joint_names:
                yaw_index = self.active_joint_names.index("thumb_cmc_yaw")
                reference_active = reference_active.copy()
                reference_active[yaw_index] = float(yaw_bias_qpos)
                posture_weights_active[yaw_index] = max(
                    posture_weights_active[yaw_index],
                    max(0.0, float(yaw_bias_weight)),
                )
            lhs = jacobian.T @ jacobian
            rhs = -(jacobian.T @ residual)
            damping = 5e-4
            lhs += damping * np.eye(len(self.active_qpos_ids))
            lhs += np.diag(posture_weights_active)
            rhs += -posture_weights_active * (active - reference_active)
            try:
                delta = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            norm = float(np.linalg.norm(delta))
            if norm > 0.28:
                delta *= 0.28 / max(norm, 1e-8)
            active = np.clip(active + 0.9 * delta, self.lower, self.upper)
            active = self._limit_thumb_tip_from_reference(active, reference_active)
            qpos[self.active_qpos_ids] = active
            self.hand_model.apply_mimic_constraints(qpos)
        return qpos

    def _limit_thumb_tip_from_reference(self, active: np.ndarray, reference_active: np.ndarray) -> np.ndarray:
        limited = np.asarray(active, dtype=np.float64).copy()
        for index, name in enumerate(self.active_joint_names):
            if name == "thumb_mcp":
                max_delta = 0.025
                limited[index] = float(
                    np.clip(
                        limited[index],
                        reference_active[index] - max_delta,
                        reference_active[index] + max_delta,
                    )
                )
                break
        return limited

    def _posture_solver_weights(self) -> np.ndarray:
        weights_by_name = {
            "thumb_cmc_yaw": 1.2e-2,
            "thumb_cmc_roll": 2.8e-2,
            "thumb_cmc_pitch": 2e-3,
            "thumb_mcp": 1.6e-1,
        }
        return np.asarray(
            [weights_by_name.get(name, 2e-3) for name in self.active_joint_names],
            dtype=np.float64,
        )

    def _posture_score_weights(self) -> np.ndarray:
        weights_by_name = {
            "thumb_cmc_yaw": 2.0,
            "thumb_cmc_roll": 3.4,
            "thumb_cmc_pitch": 0.6,
            "thumb_mcp": 8.0,
        }
        return np.asarray(
            [weights_by_name.get(name, 1.0) for name in self.active_joint_names],
            dtype=np.float64,
        )

    def _target_vector(self, human_vector: np.ndarray, human_chain_length: float) -> np.ndarray:
        vector = np.asarray(human_vector, dtype=np.float64)
        chain = max(float(human_chain_length), 1e-6)
        return vector * (self._robot_thumb_chain_length / chain)

    def _tip_vector(self) -> np.ndarray:
        return self.data.site_xpos[self.tip_site_id].copy() - self.data.xpos[self.roll_body_id].copy()

    def current_tip_vector(self) -> np.ndarray:
        return self._tip_vector().copy()

    def _tip_vector_jacobian(self) -> np.ndarray:
        mujoco = self._mujoco
        jac_tip = np.zeros((3, self.model.nv), dtype=np.float64)
        jac_origin = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacSite(self.model, self.data, jac_tip, None, int(self.tip_site_id))
        mujoco.mj_jacBody(self.model, self.data, jac_origin, None, int(self.roll_body_id))
        return (jac_tip - jac_origin)[:, self.active_dof_ids]

    def _thumb_chain_length(self) -> float:
        self.hand_model.set_qpos(self._open_qpos.copy())
        points = [
            self.data.xpos[self.roll_body_id].copy(),
            self._body_position("thumb_metacarpals"),
            self._body_position("thumb_proximal"),
            self.data.site_xpos[self.tip_site_id].copy(),
        ]
        return float(sum(np.linalg.norm(b - a) for a, b in zip(points[:-1], points[1:])))

    def _body_position(self, name: str) -> np.ndarray:
        return self.data.xpos[self._resolve_body(name)].copy()

    def _resolve_body(self, name: str) -> int:
        body_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"L20 thumb local IK body not found: {name}")
        return int(body_id)

    def _resolve_site(self, name: str) -> int:
        site_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_SITE, name)
        if site_id < 0:
            raise ValueError(f"L20 thumb local IK site not found: {name}")
        return int(site_id)

    def _active_qpos_ids(self, joint_names: tuple[str, ...]) -> np.ndarray:
        joint_index = self.hand_model.get_joint_name_to_qpos_index()
        return np.asarray([joint_index[name] for name in joint_names if name in joint_index], dtype=np.int32)

    def _active_joint_qpos_id(self, joint_name: str) -> int | None:
        joint_index = self.hand_model.get_joint_name_to_qpos_index()
        qpos_id = joint_index.get(joint_name)
        return int(qpos_id) if qpos_id is not None else None

    def _qpos_ids_to_dof_ids(self, qpos_ids: np.ndarray) -> np.ndarray:
        qpos_to_dof: dict[int, int] = {}
        for joint_id in range(self.model.njnt):
            qpos_to_dof[int(self.model.jnt_qposadr[joint_id])] = int(self.model.jnt_dofadr[joint_id])
        return np.asarray([qpos_to_dof[int(qpos_id)] for qpos_id in qpos_ids], dtype=np.int32)

    def _active_joint_ranges(self, qpos_ids: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        qpos_to_range: dict[int, tuple[float, float]] = {}
        for joint_id in range(self.model.njnt):
            qpos_id = int(self.model.jnt_qposadr[joint_id])
            low, high = self.model.jnt_range[joint_id]
            qpos_to_range[qpos_id] = (float(low), float(high))
        lower = np.asarray([qpos_to_range[int(qpos_id)][0] for qpos_id in qpos_ids], dtype=np.float64)
        upper = np.asarray([qpos_to_range[int(qpos_id)][1] for qpos_id in qpos_ids], dtype=np.float64)
        return lower, upper


class _ThumbSegmentIK:
    def __init__(
        self,
        hand_model: Any,
        *,
        adapter: Any,
        thumb_ik: _ThumbLocalIK,
        robot_segment_body: str,
        robot_open_command: list[int],
    ) -> None:
        self._mujoco = thumb_ik._mujoco
        self.hand_model = hand_model
        self.adapter = adapter
        self.model = hand_model.model
        self.data = hand_model.data
        self.origin_body_id = int(thumb_ik.roll_body_id)
        self.segment_body_id = thumb_ik._resolve_body(robot_segment_body)
        joint_index = hand_model.get_joint_name_to_qpos_index()
        self.joint_names = tuple(name for name in ("thumb_cmc_yaw", "thumb_cmc_roll") if name in joint_index)
        if not self.joint_names:
            raise ValueError("thumb segment IK needs thumb_cmc_yaw/thumb_cmc_roll joints")
        self.qpos_ids = np.asarray([joint_index[name] for name in self.joint_names], dtype=np.int32)
        self.dof_ids = thumb_ik._qpos_ids_to_dof_ids(self.qpos_ids)
        self.lower, self.upper = thumb_ik._active_joint_ranges(self.qpos_ids)

        open_qpos = adapter.sdk_range_to_qpos(list(robot_open_command))
        self.hand_model.set_qpos(open_qpos.copy())
        self.robot_open_segment_vector = self._segment_vector().copy()
        self.segment_length = max(float(np.linalg.norm(self.robot_open_segment_vector)), 1e-6)
        self._last_qpos: np.ndarray | None = None

    def solve_target(
        self,
        target_vector: np.ndarray,
        base_qpos: np.ndarray,
        *,
        damping: float,
        max_step: float,
    ) -> np.ndarray:
        qpos = np.asarray(base_qpos, dtype=np.float64).copy()
        if self._last_qpos is not None:
            qpos[self.qpos_ids] = self._last_qpos[self.qpos_ids]
        target = np.asarray(target_vector, dtype=np.float64)
        for _ in range(36):
            self.hand_model.set_qpos(qpos)
            residual = self._segment_vector() - target
            if float(np.linalg.norm(residual)) <= 8e-4:
                break
            jacobian = self._segment_jacobian()
            lhs = jacobian.T @ jacobian
            rhs = -(jacobian.T @ residual)
            lhs += float(damping) * np.eye(len(self.qpos_ids), dtype=np.float64)
            try:
                delta = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            norm = float(np.linalg.norm(delta))
            if norm > float(max_step):
                delta *= float(max_step) / max(norm, 1e-8)
            active = np.clip(qpos[self.qpos_ids] + delta, self.lower, self.upper)
            qpos[self.qpos_ids] = active
            self.hand_model.apply_mimic_constraints(qpos)
        self.hand_model.set_qpos(qpos)
        self._last_qpos = qpos.copy()
        return qpos

    def current_segment_vector(self) -> np.ndarray:
        return self._segment_vector().copy()

    def _segment_vector(self) -> np.ndarray:
        return self.data.xpos[self.segment_body_id].copy() - self.data.xpos[self.origin_body_id].copy()

    def _segment_jacobian(self) -> np.ndarray:
        jac_segment = np.zeros((3, self.model.nv), dtype=np.float64)
        jac_origin = np.zeros((3, self.model.nv), dtype=np.float64)
        self._mujoco.mj_jacBody(self.model, self.data, jac_segment, None, int(self.segment_body_id))
        self._mujoco.mj_jacBody(self.model, self.data, jac_origin, None, int(self.origin_body_id))
        return (jac_segment - jac_origin)[:, self.dof_ids]


class _ThumbPointMapper:
    def __init__(self, samples: list[dict[str, Any]], *, adapter: Any, thumb_ik: _ThumbLocalIK) -> None:
        point_samples = [sample for sample in samples if "manus_point" in sample]
        if len(point_samples) >= 2:
            samples = point_samples
        self.samples = samples
        self.adapter = adapter
        self.thumb_ik = thumb_ik
        self.manus_points = np.asarray(
            [_sample_manus_thumb_local_point(sample) for sample in samples],
            dtype=np.float64,
        )
        self.l20_points = np.asarray(
            [self._l20_point_from_command(sample["command"]) for sample in samples],
            dtype=np.float64,
        )
        self.ranges = np.maximum(np.ptp(self.manus_points, axis=0), 1e-3)
        self._plane_center: np.ndarray | None = None
        self._plane_axes: np.ndarray | None = None
        self._plane_affine: np.ndarray | None = None
        self._open_point = self._find_open_point(samples)
        self._max_open_distance = self._estimate_max_open_distance()
        self._fit_plane_affine_mapper()

    def map(self, manus_point: np.ndarray) -> np.ndarray:
        point = np.asarray(manus_point, dtype=np.float64)
        if self._plane_center is not None and self._plane_axes is not None and self._plane_affine is not None:
            uv = (point - self._plane_center) @ self._plane_axes.T
            query = np.asarray([uv[0], uv[1], 1.0], dtype=np.float64)
            return query @ self._plane_affine

        distances = np.linalg.norm((self.manus_points - point[None, :]) / self.ranges[None, :], axis=1)
        best_index = int(np.argmin(distances))
        if float(distances[best_index]) < 1e-4:
            return self.l20_points[best_index].copy()

        sigma = 0.65
        weights = np.exp(-(distances * distances) / (2.0 * sigma * sigma))
        total = float(np.sum(weights))
        if total <= 1e-9:
            return self.l20_points[best_index].copy()
        return (weights[:, None] * self.l20_points).sum(axis=0) / total

    def open_progress(self, manus_point: np.ndarray) -> float:
        if self._open_point is None:
            return 0.0
        point = np.asarray(manus_point, dtype=np.float64)
        distance = float(np.linalg.norm((point - self._open_point) / self.ranges))
        return max(0.0, min(1.0, distance / max(self._max_open_distance, 1e-6)))

    def seed_qposes(self, manus_point: np.ndarray, *, limit: int = 3) -> list[np.ndarray]:
        point = np.asarray(manus_point, dtype=np.float64)
        distances = np.linalg.norm((self.manus_points - point[None, :]) / self.ranges[None, :], axis=1)
        indexes = np.argsort(distances)[: max(1, int(limit))]
        return [self._qpos_from_command(self.samples[int(index)]["command"]) for index in indexes]

    def reference_qpos(self, manus_point: np.ndarray) -> np.ndarray:
        command = self.reference_command(manus_point)
        return self._qpos_from_command(command)

    def reference_command(self, manus_point: np.ndarray) -> dict[int, int]:
        point = np.asarray(manus_point, dtype=np.float64)
        distances = np.linalg.norm((self.manus_points - point[None, :]) / self.ranges[None, :], axis=1)
        best_index = int(np.argmin(distances))
        if float(distances[best_index]) < 1e-4:
            return dict(self.samples[best_index]["command"])

        sigma = 0.75
        weights = np.exp(-(distances * distances) / (2.0 * sigma * sigma))
        total = float(np.sum(weights))
        if total <= 1e-9:
            return dict(self.samples[best_index]["command"])

        command: dict[int, int] = {}
        for slot in THUMB_COMMAND_SLOTS:
            value = 0.0
            for sample, weight in zip(self.samples, weights):
                value += float(weight) * float(sample["command"][slot])
            command[slot] = clamp_u8(value / total)
        return command

    def _fit_plane_affine_mapper(self) -> None:
        if len(self.manus_points) < 3:
            return
        center = np.mean(self.manus_points, axis=0)
        centered = self.manus_points - center
        _, singular_values, axes = np.linalg.svd(centered, full_matrices=False)
        if len(singular_values) < 2 or float(singular_values[1]) <= 1e-6:
            return
        plane_axes = axes[:2]
        uv = centered @ plane_axes.T
        design = np.column_stack([uv, np.ones(len(uv), dtype=np.float64)])
        affine, *_ = np.linalg.lstsq(design, self.l20_points, rcond=None)
        self._plane_center = center
        self._plane_axes = plane_axes
        self._plane_affine = affine

    def _find_open_point(self, samples: list[dict[str, Any]]) -> np.ndarray | None:
        open_points = [
            _sample_manus_thumb_local_point(sample)
            for sample in samples
            if "manus_point" in sample and "open" in str(sample.get("label", "")).lower()
        ]
        if open_points:
            return np.mean(np.asarray(open_points, dtype=np.float64), axis=0)
        if len(self.manus_points):
            open_index = int(np.argmax([sample["command"].get(15, 0) for sample in samples]))
            return self.manus_points[open_index].copy()
        return None

    def _estimate_max_open_distance(self) -> float:
        if self._open_point is None or not len(self.manus_points):
            return 1.0
        distances = np.linalg.norm((self.manus_points - self._open_point[None, :]) / self.ranges[None, :], axis=1)
        return max(float(np.max(distances)), 1.0)

    def _l20_point_from_command(self, command: dict[int, int]) -> np.ndarray:
        qpos = self._qpos_from_command(command)
        self.thumb_ik.hand_model.set_qpos(qpos)
        return self.thumb_ik._tip_vector().copy()

    def _qpos_from_command(self, command: dict[int, int]) -> np.ndarray:
        full_command = list(STANDARD_OPEN_COMMAND)
        for slot, value in command.items():
            full_command[int(slot)] = clamp_u8(value)
        return self.adapter.sdk_range_to_qpos(full_command)


def _manus_thumb_local_point(landmarks: np.ndarray) -> np.ndarray:
    return _manus_thumb_local_point_at(landmarks, 4)


def _manus_thumb_local_point_at(landmarks: np.ndarray, index: int) -> np.ndarray:
    frame = _palm_frame(landmarks)
    index = max(0, min(20, int(index)))
    vector = np.asarray(landmarks[index] - landmarks[1], dtype=np.float64)
    if frame is None:
        return vector
    lateral, forward, normal = frame
    return np.asarray(
        [
            float(np.dot(vector, lateral)),
            float(np.dot(vector, forward)),
            float(np.dot(vector, normal)),
        ],
        dtype=np.float64,
    )


def _unit_vector(vector: np.ndarray) -> np.ndarray | None:
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-8:
        return None
    return vector / norm


def _unit_vector_or_default(vector: np.ndarray) -> np.ndarray:
    unit = _unit_vector(vector)
    if unit is None:
        return np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    return unit


def _rotation_between(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    source = source / max(float(np.linalg.norm(source)), 1e-8)
    target = target / max(float(np.linalg.norm(target)), 1e-8)
    cross = np.cross(source, target)
    dot = float(np.dot(source, target))
    if dot > 1.0 - 1e-8:
        return np.eye(3, dtype=np.float64)
    if dot < -1.0 + 1e-8:
        axis = np.cross(source, np.asarray([1.0, 0.0, 0.0], dtype=np.float64))
        if float(np.linalg.norm(axis)) <= 1e-8:
            axis = np.cross(source, np.asarray([0.0, 1.0, 0.0], dtype=np.float64))
        axis = axis / max(float(np.linalg.norm(axis)), 1e-8)
        return _axis_angle_rotation(axis, math.pi)
    skew = np.asarray(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + skew + skew @ skew * (1.0 / (1.0 + dot))


def _axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-8)
    x, y, z = axis
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    one_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def _sample_manus_thumb_local_point(sample: dict[str, Any]) -> np.ndarray:
    if "manus_point" in sample:
        return np.asarray(sample["manus_point"], dtype=np.float64)
    features = sample["features"]
    yaw = float(features["yaw"])
    roll = float(features["roll"])
    length = float(features.get("length", MANUS_NOMINAL_THUMB_LENGTH))
    planar = math.cos(roll)
    return np.asarray(
        [
            length * math.sin(yaw) * planar,
            length * math.cos(yaw) * planar,
            length * math.sin(roll),
        ],
        dtype=np.float64,
    )


def _thumb_chain_length(landmarks: np.ndarray) -> float:
    return float(
        np.linalg.norm(landmarks[2] - landmarks[1])
        + np.linalg.norm(landmarks[3] - landmarks[2])
        + np.linalg.norm(landmarks[4] - landmarks[3])
    )


def _calibrated_thumb_command(
    landmarks: np.ndarray,
    *,
    samples: list[dict[str, Any]],
    mode: str = "rbf",
    power: float = 6.0,
    rbf_sigma: float = 1.0,
    rbf_ridge: float = 1e-4,
    pinky_gate_start: float = 0.72,
    pinky_gate_end: float = 0.96,
) -> dict[int, int]:
    feature = _thumb_calibration_feature(landmarks)
    if mode == "snap":
        snapped = _snap_thumb_calibration(feature)
        if snapped is not None:
            return snapped
    if mode == "rbf":
        return _rbf_thumb_command(feature, samples, sigma=rbf_sigma, ridge=rbf_ridge)

    ranges = _thumb_feature_ranges(samples)
    distances = []
    for sample in samples:
        sample_feature = sample["features"]
        squared = 0.0
        for key, current_value in feature.items():
            span = ranges[key]
            delta = (current_value - float(sample_feature[key])) / span
            squared += THUMB_CALIBRATION_FEATURE_WEIGHTS[key] * delta * delta
        distances.append(math.sqrt(squared))

    best_index = int(np.argmin(np.asarray(distances, dtype=np.float64)))
    if distances[best_index] < 1e-4:
        return {
            int(slot): clamp_u8(value)
            for slot, value in samples[best_index]["command"].items()
        }

    weights = [1.0 / max(distance, 1e-6) ** max(1.0, float(power)) for distance in distances]
    if len(weights) >= 3 and mode == "idw_gate":
        weights[2] *= _thumb_pinky_gate(feature, pinky_gate_start, pinky_gate_end)
    total_weight = float(sum(weights))
    calibrated: dict[int, int] = {}
    for slot in THUMB_COMMAND_SLOTS:
        value = 0.0
        for sample, weight in zip(samples, weights):
            value += weight * float(sample["command"][slot])
        calibrated[slot] = clamp_u8(value / total_weight)
    return calibrated


def _rbf_thumb_command(
    feature: dict[str, float],
    samples: list[dict[str, Any]],
    *,
    sigma: float,
    ridge: float,
) -> dict[int, int]:
    x_train, y_train, x_query = _thumb_training_arrays(feature, samples)
    distances = np.linalg.norm(x_train - x_query[None, :], axis=1)
    best_index = int(np.argmin(distances))
    if float(distances[best_index]) < 1e-4:
        return {
            int(slot): clamp_u8(value)
            for slot, value in samples[best_index]["command"].items()
        }

    weights = np.exp(-(distances * distances) / (2.0 * sigma * sigma))
    weights = weights + ridge
    total_weight = float(np.sum(weights))
    if total_weight <= 1e-9:
        values = y_train[best_index]
    else:
        values = (weights[:, None] * y_train).sum(axis=0) / total_weight
    return {
        slot: clamp_u8(values[index])
        for index, slot in enumerate(THUMB_COMMAND_SLOTS)
    }


def _thumb_training_arrays(
    feature: dict[str, float],
    samples: list[dict[str, Any]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ranges = _thumb_feature_ranges(samples)
    mins = {
        key: min(float(sample["features"][key]) for sample in samples)
        for key in THUMB_FEATURE_KEYS
    }

    x_train = np.asarray(
        [
            [
                (float(sample["features"][key]) - mins[key]) / ranges[key]
                * math.sqrt(THUMB_CALIBRATION_FEATURE_WEIGHTS[key])
                for key in THUMB_FEATURE_KEYS
            ]
            for sample in samples
        ],
        dtype=np.float64,
    )
    y_train = np.asarray(
        [
            [float(sample["command"][slot]) for slot in THUMB_COMMAND_SLOTS]
            for sample in samples
        ],
        dtype=np.float64,
    )
    x_query = np.asarray(
        [
            (float(feature[key]) - mins[key]) / ranges[key]
            * math.sqrt(THUMB_CALIBRATION_FEATURE_WEIGHTS[key])
            for key in THUMB_FEATURE_KEYS
        ],
        dtype=np.float64,
    )
    return x_train, y_train, x_query


def _thumb_pinky_gate(feature: dict[str, float], start: float, end: float) -> float:
    open_feature = THUMB_CALIBRATION_SAMPLES[0]["features"]
    pinky_feature = THUMB_CALIBRATION_SAMPLES[2]["features"]
    tip_progress = _normalized_range(
        feature["tip"],
        float(open_feature["tip"]),
        float(pinky_feature["tip"]),
    )
    yaw_progress = _normalized_range(
        feature["yaw"],
        float(open_feature["yaw"]),
        float(pinky_feature["yaw"]),
    )
    progress = min(tip_progress, yaw_progress)
    span = max(1e-6, end - start)
    amount = max(0.0, min(1.0, (progress - start) / span))
    return amount * amount * (3.0 - 2.0 * amount)


def _normalized_range(value: float, low: float, high: float) -> float:
    span = max(1e-6, high - low)
    return max(0.0, min(1.0, (value - low) / span))


def _normalized_signed_segment(value: float, open_value: float, target_value: float) -> float:
    span = target_value - open_value
    if abs(span) <= 1e-6:
        return 0.0
    return max(0.0, min(1.0, (value - open_value) / span))


def _snap_thumb_calibration(feature: dict[str, float]) -> dict[int, int] | None:
    # The three calibrated thumb poses are intentionally sparse. For clear
    # intent regions, snap to the measured L20 command instead of averaging.
    if feature["distance"] <= 0.045 or feature["roll"] >= 0.62:
        return _thumb_sample_command(1)
    if feature["yaw"] >= 0.05 or feature["tip"] >= 0.75:
        return _thumb_sample_command(2)
    if feature["yaw"] <= -0.65 and feature["distance"] >= 0.065:
        return _thumb_sample_command(0)
    return None


def _thumb_sample_command(index: int) -> dict[int, int]:
    return {
        int(slot): clamp_u8(value)
        for slot, value in THUMB_CALIBRATION_SAMPLES[index]["command"].items()
    }


def _thumb_calibration_feature(landmarks: np.ndarray) -> dict[str, float]:
    thumb_pose = _thumb_pose_features(landmarks)
    return {
        "root": _joint_flexion_rad(landmarks[1], landmarks[2], landmarks[3]),
        "tip": _joint_flexion_rad(landmarks[2], landmarks[3], landmarks[4]),
        "yaw": float(thumb_pose["yaw_rad"]),
        "roll": float(thumb_pose["roll_rad"]),
        "distance": float(thumb_pose["thumb_index_tip_distance"]),
    }


def _thumb_feature_ranges(samples: list[dict[str, Any]]) -> dict[str, float]:
    ranges: dict[str, float] = {}
    for key in THUMB_FEATURE_KEYS:
        values = [float(sample["features"][key]) for sample in samples]
        ranges[key] = max(max(values) - min(values), 1e-3)
    return ranges


def _load_thumb_calibration_samples(path_value: str) -> list[dict[str, Any]]:
    path = Path(path_value).expanduser() if path_value else None
    if path is None or not path.exists():
        return [_normalize_thumb_sample(sample) for sample in THUMB_CALIBRATION_SAMPLES]
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    raw_samples = data.get("samples", data if isinstance(data, list) else [])
    samples = [_normalize_thumb_sample(sample) for sample in raw_samples]
    if len(samples) < 2:
        raise ValueError(f"thumb calibration needs at least 2 samples, got {len(samples)} from {path}")
    return samples


def _normalize_thumb_sample(sample: dict[str, Any]) -> dict[str, Any]:
    features_source = sample.get("features", sample.get("manus_features", {}))
    command_source = sample.get("command", sample.get("l20_command", {}))
    features = {
        key: float(features_source[key])
        for key in THUMB_FEATURE_KEYS
    }
    command = {
        slot: clamp_u8(_lookup_command_value(command_source, slot))
        for slot in THUMB_COMMAND_SLOTS
    }
    normalized: dict[str, Any] = {
        "features": features,
        "command": command,
    }
    if "manus_point" in sample:
        point = sample["manus_point"]
        if isinstance(point, (list, tuple)) and len(point) >= 3:
            normalized["manus_point"] = [float(point[index]) for index in range(3)]
    if "label" in sample:
        normalized["label"] = str(sample["label"])
    return normalized


def _lookup_command_value(command_source: Any, slot: int) -> Any:
    if isinstance(command_source, dict):
        for key in (slot, str(slot), f"{slot}"):
            if key in command_source:
                return command_source[key]
        named_key = {
            0: "thumb_base",
            5: "thumb_roll",
            10: "thumb_yaw",
            15: "thumb_tip",
        }[slot]
        if named_key in command_source:
            return command_source[named_key]
    if isinstance(command_source, (list, tuple)) and len(command_source) > slot:
        return command_source[slot]
    raise KeyError(f"missing L20 thumb command slot {slot}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManusSomehandRetargetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
