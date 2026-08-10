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

from .contact_semantics import (
    FingertipContactConfig,
    FingertipContactController,
    FingertipContactPhaseConfig,
    parse_fingertip_contact_config,
)
from .mapping import clamp_u8
from .manus_landmarks import _palm_frame
from .retarget_pipeline import (
    L20_SLOT_NAMES,
    HandFeatures,
    L20CommandAdapter,
    compute_ergonomics_flexion_targets,
    extract_hand_features,
    filter_l20_command,
    _lerp_command,
    _normalized_calibration_value,
    _scale_command_delta,
)


def _default_workspace_root() -> Path:
    env_root = os.environ.get("MANUS_L20_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / "src" / "l20_thumb_ik").exists() and (parent / "src" / "manus_l20_retarget").exists():
            return parent
        if parent.name == "install":
            candidate = parent.parent
            if (candidate / "src" / "l20_thumb_ik").exists():
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
THUMB_COMMAND_SLOTS = (0, 5, 10, 15)
THUMB_IK_COMMAND_SLOTS = (5, 10)


class ManusL20RetargetNode(Node):
    """Retarget MANUS raw skeleton nodes through the L20 thumb IK pipeline."""

    def __init__(self) -> None:
        super().__init__("manus_l20_retarget")
        workspace = _default_workspace_root()
        default_thumb_ik_root = workspace / "src" / "l20_thumb_ik"
        default_config = default_thumb_ik_root / "configs" / "retargeting" / "right" / "linkerhand_l20_right.yaml"
        default_sdk_root = default_thumb_ik_root / "third_party" / "linkerhand-python-sdk"

        self.declare_parameter("input_topic", "/manus_glove_0")
        self.declare_parameter("command_topic", "/cb_right_hand_control_cmd")
        self.declare_parameter("l20_thumb_ik_root", str(default_thumb_ik_root))
        self.declare_parameter("l20_thumb_ik_config_path", str(default_config))
        self.declare_parameter("linkerhand_sdk_root", str(default_sdk_root))
        self.declare_parameter("hand_family", "L20")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("max_delta_per_cycle", 8)
        self.declare_parameter("lowpass_alpha", 0.45)
        self.declare_parameter("watchdog_timeout_sec", 0.3)
        self.declare_parameter("reserved_command", 255)
        self.declare_parameter("start_from_open", True)
        self.declare_parameter("neutral_command", STANDARD_OPEN_COMMAND)
        self.declare_parameter("closed_command", STANDARD_FIST_COMMAND)
        self.declare_parameter("lock_neutral_slots", [5, 6, 7, 8, 9, 10, 11, 12, 13, 14])
        self.declare_parameter("finger_flexion_ergonomics_calibration_path", "")
        self.declare_parameter("root_gamma", 1.0)
        self.declare_parameter("tip_gamma", 1.0)
        self.declare_parameter("finger_yaw_calibration_path", "")
        self.declare_parameter("thumb_flexion_ergonomics_mapping_path", "")
        self.declare_parameter("thumb_flexion_root_gamma", 1.0)
        self.declare_parameter("thumb_flexion_tip_gamma", 1.0)
        self.declare_parameter("thumb_ik_debug", False)
        self.declare_parameter("thumb_segment_start", 2)
        self.declare_parameter("thumb_segment_end", 3)
        self.declare_parameter("thumb_segment_frame_path", "")
        self.declare_parameter("thumb_segment_scale", 1.0)
        self.declare_parameter("thumb_segment_damping", 8e-4)
        self.declare_parameter("thumb_segment_max_step", 0.20)
        self.declare_parameter("thumb_robot_segment_body", "thumb_metacarpals")
        self.declare_parameter("thumb_segment_robot_open_command", STANDARD_OPEN_COMMAND)
        self.declare_parameter("thumb_segment_roll_command_scale", 1.0)
        self.declare_parameter("thumb_segment_yaw_command_scale", 1.0)
        self.declare_parameter("thumb_segment_roll_command_deadzone", 0)
        self.declare_parameter("thumb_segment_roll_command_gamma", 1.0)
        self.declare_parameter("thumb_segment_roll_progress_gate_start", 0.0)
        self.declare_parameter("thumb_segment_roll_progress_gate_end", 0.0)
        self.declare_parameter("thumb_segment_yaw_progress_gate_start", 0.0)
        self.declare_parameter("thumb_segment_yaw_progress_gate_end", 0.0)
        self.declare_parameter("landmark_transform", "right_glove_to_right_retarget")
        self.declare_parameter("enable_fingertip_contact_semantics", False)
        self.declare_parameter("fingertip_contact_semantics_path", "")
        self.declare_parameter("fingertip_contact_debug", False)
        self.declare_parameter("fingertip_contact_close_orientation_completion", 0.25)
        self.declare_parameter("fingertip_contact_close_flexion_start", 0.40)
        self.declare_parameter("fingertip_contact_release_flexion_open_completion", 0.65)
        self.declare_parameter("fingertip_contact_release_orientation_gamma", 2.5)

        self._lock = threading.Lock()
        self._latest_msg: ManusGlove | None = None
        self._last_msg_time: float | None = None
        self._last_command: list[int] | None = None
        self._last_thumb_ik_debug_time = 0.0
        self._thumb_segment_debug: dict[str, Any] | None = None
        self._estop = False
        self._l20_thumb_ik_config_path = Path(
            str(self.get_parameter("l20_thumb_ik_config_path").value)
        ).expanduser().resolve()
        self._linkerhand_sdk_root = Path(str(self.get_parameter("linkerhand_sdk_root").value)).expanduser().resolve()
        self._hand_family = str(self.get_parameter("hand_family").value).upper()
        self._max_delta = int(self.get_parameter("max_delta_per_cycle").value)
        self._alpha = float(self.get_parameter("lowpass_alpha").value)
        self._watchdog_timeout = float(self.get_parameter("watchdog_timeout_sec").value)
        self._reserved_command = clamp_u8(self.get_parameter("reserved_command").value)
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
        self._lock_neutral_slots = [index for index in self._lock_neutral_slots if index not in range(6, 10)]
        self._finger_yaw_mapping: dict[str, Any] | None = None
        self._finger_flexion_ergonomics_mapping: dict[str, Any] | None = None
        self._lock_neutral_slots = [index for index in self._lock_neutral_slots if index not in (0, 5, 10, 15)]
        self._thumb_flexion_ergonomics_mapping: dict[str, Any] | None = None
        self._fingertip_contact_config: FingertipContactConfig | None = None
        self._fingertip_contact_controller: FingertipContactController | None = None
        self._fingertip_contact_debug = bool(self.get_parameter("fingertip_contact_debug").value)
        self._last_fingertip_contact_debug_time = 0.0
        self._fingertip_contact_phase = FingertipContactPhaseConfig(
            close_orientation_completion=float(
                self.get_parameter("fingertip_contact_close_orientation_completion").value
            ),
            close_flexion_start=float(self.get_parameter("fingertip_contact_close_flexion_start").value),
            release_flexion_open_completion=float(
                self.get_parameter("fingertip_contact_release_flexion_open_completion").value
            ),
            release_orientation_gamma=float(self.get_parameter("fingertip_contact_release_orientation_gamma").value),
        )
        self._thumb_flexion_root_gamma = max(
            0.05,
            float(self.get_parameter("thumb_flexion_root_gamma").value),
        )
        self._thumb_flexion_tip_gamma = max(
            0.05,
            float(self.get_parameter("thumb_flexion_tip_gamma").value),
        )
        self._thumb_ik_debug = bool(self.get_parameter("thumb_ik_debug").value)
        self._thumb_segment_start = _landmark_index_parameter(self.get_parameter("thumb_segment_start").value, 2)
        self._thumb_segment_end = _landmark_index_parameter(self.get_parameter("thumb_segment_end").value, 3)
        self._thumb_robot_segment_body = str(self.get_parameter("thumb_robot_segment_body").value)
        self._thumb_segment_robot_open_command = _command_parameter(
            self.get_parameter("thumb_segment_robot_open_command").value,
            self._neutral_command,
        )
        self._thumb_segment_frame_data = self._thumb_segment_frame_parameter(
            str(self.get_parameter("thumb_segment_frame_path").value),
        )
        if self._thumb_segment_frame_data is None:
            raise RuntimeError("thumb_segment_frame_path must contain a valid two-pose thumb IK calibration")
        self._thumb_segment_frame_rotation: np.ndarray | None = None
        self._thumb_segment_scale = max(0.01, float(self.get_parameter("thumb_segment_scale").value))
        self._thumb_segment_damping = max(1e-8, float(self.get_parameter("thumb_segment_damping").value))
        self._thumb_segment_max_step = max(1e-4, float(self.get_parameter("thumb_segment_max_step").value))
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
        self._thumb_segment_roll_progress_gate_start = max(
            0.0,
            min(1.0, float(self.get_parameter("thumb_segment_roll_progress_gate_start").value)),
        )
        self._thumb_segment_roll_progress_gate_end = max(
            0.0,
            min(1.0, float(self.get_parameter("thumb_segment_roll_progress_gate_end").value)),
        )
        self._thumb_segment_yaw_progress_gate_start = max(
            0.0,
            min(1.0, float(self.get_parameter("thumb_segment_yaw_progress_gate_start").value)),
        )
        self._thumb_segment_yaw_progress_gate_end = max(
            0.0,
            min(1.0, float(self.get_parameter("thumb_segment_yaw_progress_gate_end").value)),
        )
        self._apply_finger_yaw_calibration_path(str(self.get_parameter("finger_yaw_calibration_path").value))
        self._apply_finger_flexion_ergonomics_calibration_path(
            str(self.get_parameter("finger_flexion_ergonomics_calibration_path").value)
        )
        self._apply_thumb_flexion_ergonomics_mapping_path(
            str(self.get_parameter("thumb_flexion_ergonomics_mapping_path").value)
        )
        self._apply_fingertip_contact_semantics_path(
            str(self.get_parameter("fingertip_contact_semantics_path").value),
            enabled=bool(self.get_parameter("enable_fingertip_contact_semantics").value),
        )
        self._root_gamma = max(0.05, float(self.get_parameter("root_gamma").value))
        self._tip_gamma = max(0.05, float(self.get_parameter("tip_gamma").value))
        if bool(self.get_parameter("start_from_open").value):
            self._last_command = list(self._neutral_command)
        self._l20_command_adapter = L20CommandAdapter(
            neutral_command=self._neutral_command,
            closed_command=self._closed_command,
            reserved_command=self._reserved_command,
            lock_neutral_slots=self._lock_neutral_slots,
        )

        self._adapter = None
        self._thumb_local_ik = None
        self._thumb_segment_ik = None
        self._load_l20_thumb_ik_model_for_thumb_ik()
        self._command_pub = self.create_publisher(JointState, self.get_parameter("command_topic").value, 10)
        self.create_subscription(ManusGlove, self.get_parameter("input_topic").value, self._on_glove, 1)
        self.create_subscription(Bool, "/l20/estop", self._on_estop, 1)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._on_timer)
        self.get_logger().info(
            "MANUS -> L20 retarget node started: flexion mapping, ergonomics yaw, and two-pose thumb IK enabled"
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

    def _apply_finger_flexion_ergonomics_calibration_path(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            raise RuntimeError("finger_flexion_ergonomics_calibration_path is required")
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            raise RuntimeError(f"finger ergonomics flexion calibration not found: {path}")
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise RuntimeError(f"failed to load finger ergonomics flexion calibration {path}: {exc}") from exc

        samples = data.get("samples")
        command_data = data.get("command")
        open_sample = samples.get("natural_open") if isinstance(samples, dict) else None
        fist_sample = samples.get("four_finger_fist") if isinstance(samples, dict) else None
        root_keys = data.get("root_ergonomics_keys")
        tip_key_groups = data.get("tip_ergonomics_key_groups")
        if not (
            str(data.get("source", "")).strip().lower() == "ergonomics"
            and isinstance(command_data, dict)
            and isinstance(open_sample, dict)
            and isinstance(fist_sample, dict)
            and isinstance(root_keys, list)
            and isinstance(tip_key_groups, list)
            and len(root_keys) == len(tip_key_groups) == 4
        ):
            raise RuntimeError(f"invalid finger ergonomics flexion calibration: {path}")
        try:
            normalized_tip_groups = [[str(key) for key in group] for group in tip_key_groups]
            if any(not group for group in normalized_tip_groups):
                raise ValueError("empty tip ergonomics key group")
            mapping = {
                "root_keys": [str(key) for key in root_keys],
                "tip_key_groups": normalized_tip_groups,
                "root_open": _float_list_parameter(open_sample.get("root_value"), [], length=4),
                "root_closed": _float_list_parameter(fist_sample.get("root_value"), [], length=4),
                "tip_open": _float_list_parameter(open_sample.get("tip_value"), [], length=4),
                "tip_closed": _float_list_parameter(fist_sample.get("tip_value"), [], length=4),
            }
            if not all(len(mapping[key]) == 4 for key in ("root_open", "root_closed", "tip_open", "tip_closed")):
                raise ValueError("missing calibrated ergonomics values")
            open_command = _command_parameter(command_data.get("open_command"), self._neutral_command)
            closed_command = _command_parameter(
                command_data.get("four_finger_closed_command"),
                self._closed_command,
            )
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"invalid finger ergonomics flexion values in {path}: {exc}") from exc

        self._finger_flexion_ergonomics_mapping = mapping
        self._apply_flexion_command_calibration(
            {
                "open_command": open_command,
                "four_finger_closed_command": closed_command,
            }
        )
        self.get_logger().info(
            "loaded finger ergonomics flexion calibration "
            f"path={path}, root_keys={mapping['root_keys']}, tip_key_groups={mapping['tip_key_groups']}"
        )

    def _apply_finger_yaw_calibration_path(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            self.get_logger().warning("finger_yaw_calibration_path is empty; four-finger yaw is held at neutral")
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
        source = str(data.get("source", "")).strip().lower()
        if source != "ergonomics":
            self.get_logger().warning(f"finger yaw calibration must use source='ergonomics', got {source!r}: {path}")
            return
        ergonomics_keys = data.get("ergonomics_keys")
        if not isinstance(ergonomics_keys, list) or len(ergonomics_keys) != 4:
            self.get_logger().warning(f"finger yaw ergonomics calibration missing four ergonomics_keys: {path}")
            return
        ergonomics_keys = [str(key) for key in ergonomics_keys]
        try:
            self._finger_yaw_mapping = {
                "source": source,
                "ergonomics_keys": ergonomics_keys,
                "open_rad": _float_list_parameter(open_sample.get("yaw_rad"), [0.0] * 4, length=4),
                "close_rad": _float_list_parameter(close_sample.get("yaw_rad"), [0.0] * 4, length=4),
                "spread_rad": _float_list_parameter(spread_sample.get("yaw_rad"), [0.0] * 4, length=4),
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
            f"ergonomics_keys={ergonomics_keys}, "
            f"open_rad={np.round(self._finger_yaw_mapping['open_rad'], 5).tolist()}, "
            f"close_rad={np.round(self._finger_yaw_mapping['close_rad'], 5).tolist()}, "
            f"spread_rad={np.round(self._finger_yaw_mapping['spread_rad'], 5).tolist()}, "
            f"open_cmd={self._finger_yaw_mapping['open_cmd']}, "
            f"close_cmd={self._finger_yaw_mapping['close_cmd']}, "
            f"spread_cmd={self._finger_yaw_mapping['spread_cmd']}"
        )

    def _apply_thumb_flexion_ergonomics_mapping_path(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            raise RuntimeError("thumb_flexion_ergonomics_mapping_path is required")
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            raise RuntimeError(f"thumb ergonomics flexion mapping not found: {path}")
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise RuntimeError(f"failed to load thumb ergonomics flexion mapping {path}: {exc}") from exc

        samples = data.get("samples")
        command_data = data.get("command")
        open_sample = samples.get("thumb_natural_open") if isinstance(samples, dict) else None
        touch_sample = samples.get("thumb_pinky_root_touch") if isinstance(samples, dict) else None
        root_key = data.get("root_ergonomics_key")
        tip_keys = data.get("tip_ergonomics_keys")
        if not (
            str(data.get("source", "")).strip().lower() == "ergonomics"
            and isinstance(command_data, dict)
            and isinstance(open_sample, dict)
            and isinstance(touch_sample, dict)
            and isinstance(root_key, str)
            and isinstance(tip_keys, list)
            and tip_keys
        ):
            raise RuntimeError(f"invalid thumb ergonomics flexion mapping: {path}")
        try:
            open_command = _command_parameter(command_data.get("thumb_natural_open_command"), self._neutral_command)
            touch_command = _command_parameter(
                command_data.get("thumb_pinky_root_touch_command"),
                self._neutral_command,
            )
            mapping = {
                "root_key": root_key,
                "tip_keys": [str(key) for key in tip_keys],
                "root_open": float(open_sample["root_value"]),
                "root_touch": float(touch_sample["root_value"]),
                "tip_open": float(open_sample["tip_value"]),
                "tip_touch": float(touch_sample["tip_value"]),
                "root_open_cmd": int(open_command[0]),
                "root_touch_cmd": int(touch_command[0]),
                "tip_open_cmd": int(open_command[15]),
                "tip_touch_cmd": int(touch_command[15]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"invalid thumb ergonomics flexion values in {path}: {exc}") from exc

        self._thumb_flexion_ergonomics_mapping = mapping
        self.get_logger().info(
            "loaded thumb ergonomics flexion mapping "
            f"path={path}, root_key={mapping['root_key']}, tip_keys={mapping['tip_keys']}"
        )

    def _apply_fingertip_contact_semantics_path(self, path_value: str, *, enabled: bool) -> None:
        if not enabled:
            self.get_logger().info("fingertip contact semantics disabled by launch parameter")
            return
        path_text = str(path_value).strip()
        if not path_text:
            raise RuntimeError("fingertip_contact_semantics_path is required when contact semantics is enabled")
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            raise RuntimeError(f"fingertip contact semantics calibration not found: {path}")
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            runtime = data.get("runtime") if isinstance(data, dict) else None
            if not isinstance(runtime, dict) or not bool(runtime.get("enabled", False)):
                self.get_logger().warning(
                    f"fingertip contact semantics remains disabled by runtime.enabled=false in {path}"
                )
                return
            config = parse_fingertip_contact_config(data)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError(f"invalid fingertip contact semantics calibration {path}: {exc}") from exc
        self._fingertip_contact_config = config
        self._fingertip_contact_controller = FingertipContactController(
            config,
            self._fingertip_contact_phase,
            self._fingertip_contact_open_command,
        )
        self.get_logger().info(
            "loaded fingertip contact semantics "
            f"path={path}, pairs={list(config.profiles)}, hold={config.min_hold_sec:.3f}s, "
            f"release={config.release_hold_sec:.3f}s, filter_alpha={config.distance_filter_alpha:.3f}, "
            f"command_slew={config.command_slew_per_cycle}"
        )

    def _prepare_l20_thumb_ik_imports(self) -> None:
        thumb_ik_root = Path(str(self.get_parameter("l20_thumb_ik_root").value)).expanduser().resolve()
        src_path = thumb_ik_root / "src"
        if str(src_path) not in sys.path:
            sys.path.insert(0, str(src_path))

    def _load_l20_thumb_ik_model_for_thumb_ik(self) -> None:
        self._prepare_l20_thumb_ik_imports()
        from l20_ik_core.infrastructure.config_loader import load_retargeting_config
        from l20_ik_core.infrastructure.controllers.adapters import LinkerHandModelAdapter
        from l20_ik_core.infrastructure.hand_model import HandModel

        config = load_retargeting_config(str(self._l20_thumb_ik_config_path))
        hand_model = HandModel(config.hand.mjcf_path)
        self._adapter = LinkerHandModelAdapter(
            hand_model,
            family=self._hand_family,
            hand_side=config.hand.side,
            sdk_root=str(self._linkerhand_sdk_root),
        )
        self._initialize_thumb_segment_ik(hand_model)
        self.get_logger().info(
            f"loaded L20 thumb IK model config={self._l20_thumb_ik_config_path}, "
            f"family={self._hand_family}, sdk_root={self._linkerhand_sdk_root}"
        )

    def _initialize_thumb_segment_ik(self, hand_model: Any) -> None:
        if self._adapter is None:
            raise RuntimeError("thumb segment IK requested before LinkerHand adapter initialization")
        self._thumb_local_ik = _ThumbLocalIK(hand_model)
        self._thumb_segment_ik = _ThumbSegmentIK(
            hand_model,
            adapter=self._adapter,
            thumb_ik=self._thumb_local_ik,
            robot_segment_body=self._thumb_robot_segment_body,
            robot_open_command=self._thumb_segment_robot_open_command,
        )
        self._initialize_thumb_segment_frame_rotation()

    def _on_glove(self, msg: ManusGlove) -> None:
        with self._lock:
            self._latest_msg = msg
            self._last_msg_time = monotonic()

    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)
        if self._estop:
            self._reset_fingertip_contact_semantics()
            self._last_command = list(self._neutral_command)
            self._publish(self._last_command)

    def _on_timer(self) -> None:
        with self._lock:
            msg = self._latest_msg
            last_msg_time = self._last_msg_time

        if self._estop:
            self._reset_fingertip_contact_semantics()
            self._publish(self._neutral_command)
            return

        if msg is None:
            return
        if last_msg_time is not None and monotonic() - last_msg_time > self._watchdog_timeout:
            self._reset_fingertip_contact_semantics()
            self._last_command = self._filter_command(self._neutral_command)
            self._publish(self._last_command)
            return

        try:
            command = self._command_from_manus(msg)
        except Exception as exc:
            self.get_logger().warning(f"failed to retarget MANUS frame: {exc}")
            return

        filtered_command = self._filter_command(command)
        self._debug_thumb_segment_publish(command, filtered_command)
        self._last_command = filtered_command
        self._publish(self._last_command)

    def _reset_fingertip_contact_semantics(self) -> None:
        if self._fingertip_contact_controller is not None:
            self._fingertip_contact_controller.reset()

    def _thumb_segment_frame_parameter(self, path_value: str) -> dict[str, Any] | None:
        path_text = str(path_value).strip()
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        if not path.exists():
            self.get_logger().info(
                f"thumb segment frame file not found: {path}"
            )
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            open_vector = _optional_vector3_parameter(data.get("manus_open_vector"))
            touch_vector = _optional_vector3_parameter(data.get("manus_touch_vector"))
            if open_vector is None or touch_vector is None:
                raise ValueError("missing manus_open_vector/manus_touch_vector")
            segment_start = data.get("segment_start")
            segment_end = data.get("segment_end")
            if segment_start is not None and int(segment_start) != self._thumb_segment_start:
                self.get_logger().warning(
                    f"thumb segment frame start={segment_start} does not match "
                    f"thumb_segment_start={self._thumb_segment_start}"
                )
            if segment_end is not None and int(segment_end) != self._thumb_segment_end:
                self.get_logger().warning(
                    f"thumb segment frame end={segment_end} does not match "
                    f"thumb_segment_end={self._thumb_segment_end}"
                )
            robot_open_command = _command_parameter(
                data.get("robot_open_command"),
                self._thumb_segment_robot_open_command,
            )
            robot_touch_command = _command_parameter(
                data.get("robot_touch_command"),
                robot_open_command,
            )
            self.get_logger().info(
                f"loaded thumb segment frame path={path}, "
                f"manus_open={np.round(open_vector, 6).tolist()}, "
                f"manus_touch={np.round(touch_vector, 6).tolist()}"
            )
            return {
                "path": path,
                "manus_open_vector": open_vector,
                "manus_touch_vector": touch_vector,
                "robot_open_command": robot_open_command,
                "robot_touch_command": robot_touch_command,
            }
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(
                f"failed to load thumb segment frame from {path}: {exc}"
            )
            return None

    def _initialize_thumb_segment_frame_rotation(self) -> None:
        if self._thumb_segment_frame_data is None or self._thumb_segment_ik is None or self._adapter is None:
            return
        data = self._thumb_segment_frame_data
        manus_open = np.asarray(data["manus_open_vector"], dtype=np.float64)
        manus_touch = np.asarray(data["manus_touch_vector"], dtype=np.float64)
        robot_open = self._robot_segment_vector_for_command(data["robot_open_command"])
        robot_touch = self._robot_segment_vector_for_command(data["robot_touch_command"])
        rotation = _frame_rotation_from_two_vectors(manus_open, manus_touch, robot_open, robot_touch)
        if rotation is None:
            raise RuntimeError("thumb segment frame is degenerate")
        self._thumb_segment_frame_rotation = rotation
        self.get_logger().info(
            "initialized thumb segment frame alignment "
            f"robot_open={np.round(robot_open, 6).tolist()}, "
            f"robot_touch={np.round(robot_touch, 6).tolist()}"
        )

    def _robot_segment_vector_for_command(self, command: list[int]) -> np.ndarray:
        if self._thumb_segment_ik is None or self._adapter is None:
            raise RuntimeError("thumb segment frame requested before IK initialization")
        qpos = self._adapter.sdk_range_to_qpos(command)
        self._thumb_segment_ik.hand_model.set_qpos(qpos.copy())
        return self._thumb_segment_ik.current_segment_vector()

    def _command_from_manus(self, msg: ManusGlove) -> list[int]:
        features = extract_hand_features(
            msg,
            landmark_transform=str(self.get_parameter("landmark_transform").value),
        )
        return self._command_from_ergonomics(features)

    def _command_from_ergonomics(
        self,
        features: HandFeatures,
    ) -> list[int]:
        landmarks = features.landmarks
        ergonomics_mapping = self._finger_flexion_ergonomics_mapping
        if ergonomics_mapping is None:
            raise RuntimeError("finger ergonomics flexion calibration was not loaded")
        targets = compute_ergonomics_flexion_targets(
            features.ergonomics,
            root_keys=ergonomics_mapping["root_keys"],
            tip_key_groups=ergonomics_mapping["tip_key_groups"],
            root_open_values=ergonomics_mapping["root_open"],
            root_closed_values=ergonomics_mapping["root_closed"],
            tip_open_values=ergonomics_mapping["tip_open"],
            tip_closed_values=ergonomics_mapping["tip_closed"],
            root_gamma=self._root_gamma,
            tip_gamma=self._tip_gamma,
        )
        command = self._l20_command_adapter.command_from_targets(targets)
        self._apply_finger_yaw(command, features.ergonomics)
        self._apply_thumb_flexion_mapping(command, features.ergonomics)
        self._apply_thumb_ik(command, landmarks)
        self._apply_fingertip_contact_semantics(command, features.skeleton)
        self._l20_command_adapter.apply_reserved_slots(command)
        self._l20_command_adapter.apply_neutral_locks(command)
        return command

    def _apply_finger_yaw(
        self,
        command: list[int],
        ergonomics: dict[str, float],
    ) -> None:
        if self._finger_yaw_mapping is not None:
            keys = self._finger_yaw_mapping["ergonomics_keys"]
            try:
                yaw_angles = np.asarray([float(ergonomics[str(key)]) for key in keys], dtype=np.float64)
            except (KeyError, TypeError, ValueError):
                return
            open_rad = self._finger_yaw_mapping["open_rad"]
            close_rad = self._finger_yaw_mapping["close_rad"]
            spread_rad = self._finger_yaw_mapping["spread_rad"]
            open_cmd = self._finger_yaw_mapping["open_cmd"]
            close_cmd = self._finger_yaw_mapping["close_cmd"]
            spread_cmd = self._finger_yaw_mapping["spread_cmd"]
            for local_index, angle in enumerate(yaw_angles):
                slot = 6 + local_index
                open_value = int(open_cmd[local_index])
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
                    yaw_command = _lerp_command(
                        open_value,
                        int(close_cmd[local_index]),
                        close_amount,
                    )
                else:
                    yaw_command = _lerp_command(
                        open_value,
                        int(spread_cmd[local_index]),
                        spread_amount,
                    )
                command[slot] = yaw_command
            return

    def _apply_thumb_flexion_mapping(
        self,
        command: list[int],
        ergonomics: dict[str, float],
    ) -> None:
        ergonomics_mapping = self._thumb_flexion_ergonomics_mapping
        if ergonomics_mapping is None:
            raise RuntimeError("thumb ergonomics flexion mapping was not loaded")
        root_value = float(ergonomics[ergonomics_mapping["root_key"]])
        tip_value = sum(float(ergonomics[key]) for key in ergonomics_mapping["tip_keys"])
        root_amount = _normalized_calibration_value(
            root_value,
            ergonomics_mapping["root_open"],
            ergonomics_mapping["root_touch"],
            self._thumb_flexion_root_gamma,
        )
        tip_amount = _normalized_calibration_value(
            tip_value,
            ergonomics_mapping["tip_open"],
            ergonomics_mapping["tip_touch"],
            self._thumb_flexion_tip_gamma,
        )
        command[0] = _lerp_command(
            ergonomics_mapping["root_open_cmd"],
            ergonomics_mapping["root_touch_cmd"],
            root_amount,
        )
        command[15] = _lerp_command(
            ergonomics_mapping["tip_open_cmd"],
            ergonomics_mapping["tip_touch_cmd"],
            tip_amount,
        )

    def _apply_thumb_ik(self, command: list[int], landmarks: np.ndarray) -> None:
        ik_command = self._thumb_segment_ik_command(command, landmarks)
        values = {slot: ik_command[slot] for slot in THUMB_IK_COMMAND_SLOTS}
        raw_values = dict(values)
        if self._thumb_segment_debug is not None:
            self._thumb_segment_debug["raw_cmd"] = [raw_values[index] for index in THUMB_IK_COMMAND_SLOTS]
            self._thumb_segment_debug["smooth_cmd"] = [values[index] for index in THUMB_IK_COMMAND_SLOTS]
        for slot, value in values.items():
            command[slot] = value

    def _apply_fingertip_contact_semantics(self, command: list[int], skeleton: Any) -> None:
        controller = self._fingertip_contact_controller
        if controller is None:
            return
        try:
            updated_command, decision = controller.apply(command, skeleton, monotonic())
        except (AttributeError, TypeError, ValueError) as exc:
            controller.reset()
            if self._fingertip_contact_debug:
                self.get_logger().warning(f"fingertip contact frame ignored: {exc}")
            return

        command[:] = updated_command
        if self._fingertip_contact_debug:
            self._debug_fingertip_contact_decision(decision)

    def _debug_fingertip_contact_decision(self, decision: Any) -> None:
        now = monotonic()
        if decision.event is None and now - self._last_fingertip_contact_debug_time < 0.20:
            return
        self._last_fingertip_contact_debug_time = now
        feature = decision.features.get(decision.pair) if decision.pair is not None else None
        if feature is None and decision.features:
            feature = next(iter(decision.features.values()))
        feature_text = ""
        if feature is not None:
            feature_text = (
                f" distance_raw={feature.distance_raw:.4f} distance_filtered={feature.distance_filtered:.4f} "
                f"progress={feature.progress:.3f} velocity={feature.velocity:.4f}"
            )
        slots = {slot: round(value, 3) for slot, value in decision.slot_activations.items()}
        base_slots = dict(decision.base_command_slots)
        desired_slots = dict(decision.desired_command_slots)
        command_slots = dict(decision.command_slots)
        prefix = f"fingertip_contact {decision.event}" if decision.event else "fingertip_contact"
        self.get_logger().info(
            f"{prefix} state={decision.state} pair={decision.pair} activation={decision.activation:.3f} "
            f"phase={decision.phase:.3f} phase_target={decision.phase_target:.3f}"
            f"{feature_text} slot_activation={slots} base_slots={base_slots} "
            f"desired_slots={desired_slots} command_slots={command_slots}"
        )

    def _fingertip_contact_open_command(self, slot: int) -> int:
        if self._thumb_flexion_ergonomics_mapping is not None:
            if slot == 0:
                return clamp_u8(self._thumb_flexion_ergonomics_mapping["root_open_cmd"])
            if slot == 15:
                return clamp_u8(self._thumb_flexion_ergonomics_mapping["tip_open_cmd"])
        return clamp_u8(self._neutral_command[slot])

    def _thumb_segment_ik_command(self, base_command: list[int], landmarks: np.ndarray) -> list[int]:
        if self._adapter is None or self._thumb_segment_ik is None:
            raise RuntimeError("thumb segment IK requested but L20 thumb IK model was not loaded")
        base_qpos = self._adapter.sdk_range_to_qpos(base_command)
        target_vector = self._thumb_segment_target_vector(landmarks)
        thumb_progress = self._thumb_segment_motion_progress(landmarks)
        roll_progress_gate = self._thumb_segment_progress_gate(
            thumb_progress,
            self._thumb_segment_roll_progress_gate_start,
            self._thumb_segment_roll_progress_gate_end,
        )
        yaw_progress_gate = self._thumb_segment_progress_gate(
            thumb_progress,
            self._thumb_segment_yaw_progress_gate_start,
            self._thumb_segment_yaw_progress_gate_end,
        )
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
            self._thumb_segment_roll_command_scale * roll_progress_gate,
            self._thumb_segment_roll_command_gamma,
            self._thumb_segment_roll_command_deadzone,
        )
        command[10] = _scale_command_delta(
            command[10],
            self._thumb_segment_robot_open_command[10],
            self._thumb_segment_yaw_command_scale * yaw_progress_gate,
        )
        self._thumb_segment_debug = {
            "start": self._thumb_segment_start,
            "end": self._thumb_segment_end,
            "map": "raw",
            "align": "two_pose_frame",
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
            "progress_gates": [
                None if thumb_progress is None else round(float(thumb_progress), 3),
                round(float(roll_progress_gate), 3),
                round(float(yaw_progress_gate), 3),
            ],
            "roll_yaw_ik": ik_roll_yaw_command,
            "base_cmd": [base_command[index] for index in THUMB_COMMAND_SLOTS],
            "ik_cmd": [command[index] for index in THUMB_COMMAND_SLOTS],
        }
        for index in range(11, 15):
            command[index] = self._reserved_command
        return command

    def _debug_thumb_segment_publish(self, raw_command: list[int], filtered_command: list[int]) -> None:
        if not self._thumb_ik_debug:
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
            f"progress_gates={debug['progress_gates']} "
            f"roll_yaw_ik={debug['roll_yaw_ik']} "
            f"base_cmd={debug['base_cmd']} ik_cmd={debug['ik_cmd']} "
            f"raw_cmd={[raw_command[index] for index in THUMB_IK_COMMAND_SLOTS]} "
            f"smooth_cmd={debug.get('smooth_cmd')} "
            f"pub_cmd={[filtered_command[index] for index in THUMB_IK_COMMAND_SLOTS]}"
        )

    def _thumb_segment_target_vector(self, landmarks: np.ndarray) -> np.ndarray:
        if self._thumb_segment_ik is None:
            raise RuntimeError("thumb segment IK target requested before initialization")
        vector = self._thumb_segment_source_vector(landmarks)
        vector = self._align_thumb_segment_open(vector)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-8:
            return self._thumb_segment_ik.robot_open_segment_vector.copy()
        return vector / norm * self._thumb_segment_ik.segment_length * self._thumb_segment_scale

    def _thumb_segment_source_vector(self, landmarks: np.ndarray) -> np.ndarray:
        start = _manus_thumb_local_point_at(landmarks, self._thumb_segment_start)
        end = _manus_thumb_local_point_at(landmarks, self._thumb_segment_end)
        return end if self._thumb_segment_start == self._thumb_segment_end else end - start

    def _thumb_segment_motion_progress(self, landmarks: np.ndarray) -> float | None:
        if self._thumb_segment_frame_data is not None:
            open_vector = np.asarray(self._thumb_segment_frame_data["manus_open_vector"], dtype=np.float64)
            touch_vector = np.asarray(self._thumb_segment_frame_data["manus_touch_vector"], dtype=np.float64)
            span = touch_vector - open_vector
            denominator = float(np.dot(span, span))
            if denominator > 1e-10:
                vector = self._thumb_segment_source_vector(landmarks)
                amount = float(np.dot(vector - open_vector, span) / denominator)
                return max(0.0, min(1.0, amount))
        return None

    @staticmethod
    def _thumb_segment_progress_gate(progress: float | None, start: float, end: float) -> float:
        if progress is None or end <= start:
            return 1.0
        if progress <= start:
            return 0.0
        if progress >= end:
            return 1.0
        return (float(progress) - start) / max(1e-6, end - start)

    def _align_thumb_segment_open(self, vector: np.ndarray) -> np.ndarray:
        vector = np.asarray(vector, dtype=np.float64)
        if self._thumb_segment_ik is None:
            return vector
        if self._thumb_segment_frame_rotation is None:
            raise RuntimeError("thumb segment frame rotation was not initialized")
        return self._thumb_segment_frame_rotation @ vector

    def _filter_command(self, command: list[int]) -> list[int]:
        return filter_l20_command(
            command,
            last_command=self._last_command,
            max_delta_per_cycle=self._max_delta,
            lowpass_alpha=self._alpha,
        )

    def _publish(self, command: list[int]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = L20_SLOT_NAMES
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


def _frame_rotation_from_two_vectors(
    source_open: np.ndarray,
    source_touch: np.ndarray,
    target_open: np.ndarray,
    target_touch: np.ndarray,
) -> np.ndarray | None:
    source_frame = _basis_from_open_touch(source_open, source_touch)
    target_frame = _basis_from_open_touch(target_open, target_touch)
    if source_frame is None or target_frame is None:
        return None
    return target_frame @ source_frame.T


def _basis_from_open_touch(open_vector: np.ndarray, touch_vector: np.ndarray) -> np.ndarray | None:
    primary = _unit_vector(open_vector)
    if primary is None:
        return None
    touch = np.asarray(touch_vector, dtype=np.float64)
    lateral = touch - float(np.dot(touch, primary)) * primary
    lateral = _unit_vector(lateral)
    if lateral is None:
        return None
    normal = _unit_vector(np.cross(primary, lateral))
    if normal is None:
        return None
    lateral = _unit_vector(np.cross(normal, primary))
    if lateral is None:
        return None
    return np.column_stack((primary, lateral, normal))


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


def _normalized_signed_segment(value: float, open_value: float, target_value: float) -> float:
    span = target_value - open_value
    if abs(span) <= 1e-6:
        return 0.0
    return max(0.0, min(1.0, (value - open_value) / span))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManusL20RetargetNode()
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
