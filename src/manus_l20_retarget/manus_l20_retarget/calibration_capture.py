"""Calibration capture and persistence services used by the Qt workflow."""

from __future__ import annotations

import math
import statistics
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node

from .contact_semantics import (
    FINGERTIP_CONTACT_FINGERS,
    FINGERTIP_CONTACT_SLOTS,
    thumb_fingertip_distance_ratios,
)
from .manus_landmarks import manus_raw_nodes_to_hand_skeleton, manus_raw_nodes_to_mediapipe_landmarks
from .manus_l20_retarget_node import (
    STANDARD_OPEN_COMMAND,
    _command_parameter,
    _default_workspace_root,
    _manus_thumb_local_point_at,
)


DEFAULT_L20_OPEN_COMMAND = [
    255, 255, 255, 255, 255,
    128, 163, 129, 86, 62, 255,
    255, 255, 255, 255,
    255, 255, 255, 255, 255,
]
DEFAULT_L20_FOUR_FINGER_CLOSED_COMMAND = [
    187, 13, 0, 5, 15,
    128, 128, 128, 128, 128, 255,
    255, 255, 255, 255,
    110, 12, 12, 2, 9,
]
DEFAULT_L20_FINGER_CLOSE_COMMAND = [
    255, 255, 255, 255, 255,
    128, 60, 85, 136, 166, 255,
    255, 255, 255, 255,
    255, 255, 255, 255, 255,
]
DEFAULT_L20_FINGER_SPREAD_COMMAND = [
    255, 255, 255, 255, 255,
    128, 255, 169, 46, 0, 255,
    255, 255, 255, 255,
    255, 255, 255, 255, 255,
]
DEFAULT_THUMB_NATURAL_OPEN_COMMAND = [
    99, 255, 255, 255, 255,
    0, 163, 129, 86, 62, 222,
    255, 255, 255, 255,
    100, 255, 255, 255, 255,
]
FINGER_NAMES = ("index", "middle", "ring", "pinky")
FINGER_FLEXION_ROOT_ERGONOMICS_KEYS = (
    "IndexMCPStretch",
    "MiddleMCPStretch",
    "RingMCPStretch",
    "PinkyMCPStretch",
)
FINGER_FLEXION_TIP_ERGONOMICS_KEY_GROUPS = (
    ("IndexPIPStretch", "IndexDIPStretch"),
    ("MiddlePIPStretch", "MiddleDIPStretch"),
    ("RingPIPStretch", "RingDIPStretch"),
    ("PinkyPIPStretch", "PinkyDIPStretch"),
)
THUMB_FLEXION_ROOT_ERGONOMICS_KEY = "ThumbMCPStretch"
THUMB_FLEXION_TIP_ERGONOMICS_KEYS = ("ThumbPIPStretch", "ThumbDIPStretch")
ERGONOMICS_YAW_TOKENS = ("spread", "abduction", "adduction", "abd", "add", "yaw")
L20_COMMAND_SLOT_COMMENTS = (
    "0  Thumb Base / 拇指根部弯曲",
    "1  Index Finger Base / 食指根部弯曲",
    "2  Middle Finger Base / 中指根部弯曲",
    "3  Ring Finger Base / 无名指根部弯曲",
    "4  Pinky Finger Base / 小指根部弯曲",
    "5  Thumb Roll / 拇指 roll",
    "6  Index Finger Yaw / 食指侧摆",
    "7  Middle Finger Yaw / 中指侧摆",
    "8  Ring Finger Yaw / 无名指侧摆",
    "9  Pinky Finger Yaw / 小指侧摆",
    "10 Thumb Yaw / 拇指 yaw",
    "11 Reserved / 保留位",
    "12 Reserved / 保留位",
    "13 Reserved / 保留位",
    "14 Reserved / 保留位",
    "15 Thumb Tip / 拇指指尖弯曲",
    "16 Index Finger Tip / 食指指尖弯曲",
    "17 Middle Finger Tip / 中指指尖弯曲",
    "18 Ring Finger Tip / 无名指指尖弯曲",
    "19 Pinky Finger Tip / 小指指尖弯曲",
)
FULL_CALIBRATION_POSES = (
    ("natural_open", "自然张开；同时作为四指 open、yaw open、拇指 open、拇指 IK open"),
    ("four_finger_fist", "四指完全弯曲握拳；只用于四指弯曲 closed"),
    ("finger_close", "四指并拢；只用于四指 yaw close"),
    ("finger_spread", "四指外展；只用于四指 yaw spread"),
    ("thumb_pinky_root_touch", "大拇指触碰小拇指指根；同时用于拇指弯曲 touch、拇指 IK touch"),
)
FINGERTIP_CONTACT_POSES = (
    ("natural_open", "自然张开，拇指远离四指"),
    ("thumb_index_tip_touch", "拇指指尖触碰食指指尖"),
)


class FullCalibrationCapture(Node):
    def __init__(
        self,
        glove_topic: str,
        transform: str,
        segment_start: int,
        segment_end: int,
    ) -> None:
        super().__init__("manus_l20_full_calibration_capture")
        self._transform = transform
        self._segment_start = segment_start
        self._segment_end = segment_end
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[dict[str, Any]] = []
        self.create_subscription(ManusGlove, glove_topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=self._transform,
        )
        start = _manus_thumb_local_point_at(landmarks, self._segment_start)
        end = _manus_thumb_local_point_at(landmarks, self._segment_end)
        vector = end if self._segment_start == self._segment_end else end - start
        ergonomics = {str(entry.type): float(entry.value) for entry in msg.ergonomics if entry.type}
        sample = {
            "thumb_segment_vector": [float(value) for value in vector],
            "yaw": ergonomics,
        }
        with self._lock:
            if self._collecting:
                self._samples.append(sample)

    def capture(self, duration_sec: float) -> dict[str, Any]:
        with self._lock:
            self._samples = []
            self._collecting = True
        time.sleep(duration_sec)
        with self._lock:
            self._collecting = False
            samples = list(self._samples)
        if not samples:
            raise RuntimeError("no MANUS glove samples captured")
        return {
            "thumb_segment_vector": _mean_vector([sample["thumb_segment_vector"] for sample in samples]),
            "yaw": _mean_ergonomics(samples),
        }


class FingertipContactCapture(Node):
    """Capture raw-skeleton thumb-to-fingertip distance samples without commanding L20."""

    def __init__(self, glove_topic: str, transform: str) -> None:
        super().__init__("manus_l20_fingertip_contact_calibration_capture")
        self._transform = transform
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[dict[str, float]] = []
        self._discarded_frames = 0
        self._last_invalid_frame_warning_sec = float("-inf")
        self.create_subscription(ManusGlove, glove_topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        try:
            skeleton = manus_raw_nodes_to_hand_skeleton(msg.raw_nodes, transform=self._transform)
            ratios = thumb_fingertip_distance_ratios(skeleton)
        except (AttributeError, TypeError, ValueError) as exc:
            now_sec = time.monotonic()
            with self._lock:
                if not self._collecting:
                    return
                self._discarded_frames += 1
                warn = now_sec - self._last_invalid_frame_warning_sec >= 1.0
                if warn:
                    self._last_invalid_frame_warning_sec = now_sec
            if warn:
                self.get_logger().warning(
                    f"ignoring incomplete MANUS raw skeleton frame during fingertip-contact capture: {exc}"
                )
            return
        with self._lock:
            if self._collecting:
                self._samples.append(ratios)

    def capture(self, duration_sec: float) -> dict[str, list[float]]:
        with self._lock:
            self._samples = []
            self._discarded_frames = 0
            self._collecting = True
        time.sleep(duration_sec)
        with self._lock:
            self._collecting = False
            samples = list(self._samples)
            discarded_frames = self._discarded_frames
        if not samples:
            raise RuntimeError(
                "no complete MANUS raw-skeleton samples captured "
                f"in {duration_sec:.1f}s; discarded {discarded_frames} incomplete frames. "
                "Verify that the MANUS glove is connected and raw skeleton streaming is enabled."
            )
        return {
            finger: [float(sample[finger]) for sample in samples if finger in sample]
            for finger in FINGERTIP_CONTACT_FINGERS
        }


def _mean_vector(samples: list[list[float]]) -> list[float]:
    length = min(len(sample) for sample in samples)
    return [
        round(float(statistics.fmean(sample[index] for sample in samples)), 6)
        for index in range(length)
    ]


def _mean_ergonomics(samples: list[dict[str, Any]]) -> dict[str, float]:
    yaw_samples = [sample.get("yaw") for sample in samples if isinstance(sample.get("yaw"), dict)]
    keys = sorted({str(key) for sample in yaw_samples for key in sample})
    return {
        key: round(float(statistics.fmean(float(sample[key]) for sample in yaw_samples if key in sample)), 6)
        for key in keys
    }


def _existing_command(path: str) -> dict[str, Any] | None:
    output_path = Path(path).expanduser()
    if not output_path.exists():
        return None
    try:
        with output_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return None
    command = data.get("command")
    return command if isinstance(command, dict) else None


def _existing_top_level_command(path: str, key: str, fallback: list[int]) -> list[int]:
    output_path = Path(path).expanduser()
    if not output_path.exists():
        return list(fallback)
    try:
        with output_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return list(fallback)
    return _command_parameter(data.get(key), fallback)


def _build_finger_yaw_ergonomics_calibration(
    samples: dict[str, dict[str, float]],
    output_path: str,
    ergonomics_keys: list[str] | None = None,
) -> dict[str, Any]:
    selected_keys = ergonomics_keys or _select_ergonomics_yaw_keys(samples)
    command = _existing_command(output_path) or {
        "natural_open_command": DEFAULT_L20_OPEN_COMMAND,
        "finger_close_command": DEFAULT_L20_FINGER_CLOSE_COMMAND,
        "finger_spread_command": DEFAULT_L20_FINGER_SPREAD_COMMAND,
    }
    selected_samples = {
        label: {
            "yaw_rad": [round(float(sample.get(key, 0.0)), 6) for key in selected_keys],
            "ergonomics": sample,
        }
        for label, sample in samples.items()
    }
    return {
        "schema": "manus_l20.finger_yaw_calibration.v1",
        "finger_order": list(FINGER_NAMES),
        "source": "ergonomics",
        "source_mode": "ergonomics_auto" if not ergonomics_keys else "ergonomics_keys",
        "ergonomics_keys": selected_keys,
        "ergonomics_source_ranges": _ergonomics_key_ranges(samples, selected_keys),
        "command": command,
        "samples": selected_samples,
    }


def _select_ergonomics_yaw_keys(samples: dict[str, dict[str, float]]) -> list[str]:
    keys = sorted({key for sample in samples.values() for key in sample})
    if not keys:
        raise RuntimeError("MANUS ergonomics samples are empty")
    selected: list[str] = []
    for finger in FINGER_NAMES:
        candidates = [
            key for key in keys
            if finger in _normalized_key(key) and any(token in _normalized_key(key) for token in ERGONOMICS_YAW_TOKENS)
        ]
        if not candidates:
            candidates = [key for key in keys if finger in _normalized_key(key)]
        if not candidates:
            raise RuntimeError(f"no MANUS ergonomics key found for finger yaw: {finger}; available keys={keys}")
        selected.append(max(candidates, key=lambda key: _ergonomics_key_range(samples, key)))
    return selected


def _ergonomics_key_ranges(samples: dict[str, dict[str, float]], keys: list[str]) -> dict[str, float]:
    return {key: round(_ergonomics_key_range(samples, key), 6) for key in keys}


def _ergonomics_key_range(samples: dict[str, dict[str, float]], key: str) -> float:
    open_value = float(samples.get("natural_open", {}).get(key, 0.0))
    close_value = float(samples.get("finger_close", {}).get(key, open_value))
    spread_value = float(samples.get("finger_spread", {}).get(key, open_value))
    return max(abs(close_value - open_value), abs(spread_value - open_value))


def _normalized_key(key: str) -> str:
    return "".join(ch.lower() for ch in str(key) if ch.isalnum())


def _ergonomics_values(sample: dict[str, float], keys: tuple[str, ...]) -> list[float]:
    missing = [key for key in keys if key not in sample]
    if missing:
        raise RuntimeError(f"MANUS ergonomics sample is missing keys: {missing}")
    return [round(float(sample[key]), 6) for key in keys]


def _ergonomics_group_values(
    sample: dict[str, float],
    key_groups: tuple[tuple[str, ...], ...],
) -> list[float]:
    values: list[float] = []
    for keys in key_groups:
        missing = [key for key in keys if key not in sample]
        if missing:
            raise RuntimeError(f"MANUS ergonomics sample is missing keys: {missing}")
        values.append(round(sum(float(sample[key]) for key in keys), 6))
    return values


def _build_finger_flexion_ergonomics_calibration(
    samples: dict[str, dict[str, float]],
    output_path: str,
) -> dict[str, Any]:
    open_sample = samples["natural_open"]
    fist_sample = samples["four_finger_fist"]
    command = _existing_command(output_path) or {
        "open_command": DEFAULT_L20_OPEN_COMMAND,
        "four_finger_closed_command": DEFAULT_L20_FOUR_FINGER_CLOSED_COMMAND,
    }
    return {
        "schema": "manus_l20.finger_flexion_ergonomics.v1",
        "finger_order": list(FINGER_NAMES),
        "source": "ergonomics",
        "root_ergonomics_keys": list(FINGER_FLEXION_ROOT_ERGONOMICS_KEYS),
        "tip_ergonomics_key_groups": [list(keys) for keys in FINGER_FLEXION_TIP_ERGONOMICS_KEY_GROUPS],
        "command": command,
        "samples": {
            "natural_open": {
                "root_value": _ergonomics_values(open_sample, FINGER_FLEXION_ROOT_ERGONOMICS_KEYS),
                "tip_value": _ergonomics_group_values(open_sample, FINGER_FLEXION_TIP_ERGONOMICS_KEY_GROUPS),
                "ergonomics": open_sample,
            },
            "four_finger_fist": {
                "root_value": _ergonomics_values(fist_sample, FINGER_FLEXION_ROOT_ERGONOMICS_KEYS),
                "tip_value": _ergonomics_group_values(fist_sample, FINGER_FLEXION_TIP_ERGONOMICS_KEY_GROUPS),
                "ergonomics": fist_sample,
            },
        },
    }


def _build_thumb_flexion_ergonomics_calibration(
    samples: dict[str, dict[str, float]],
    output_path: str,
) -> dict[str, Any]:
    open_sample = samples["thumb_natural_open"]
    touch_sample = samples["thumb_pinky_root_touch"]
    command = _existing_command(output_path) or {
        "thumb_natural_open_command": DEFAULT_THUMB_NATURAL_OPEN_COMMAND,
        "thumb_pinky_root_touch_command": None,
    }
    return {
        "schema": "manus_l20.thumb_flexion_ergonomics.v1",
        "finger": "thumb",
        "source": "ergonomics",
        "root_ergonomics_key": THUMB_FLEXION_ROOT_ERGONOMICS_KEY,
        "tip_ergonomics_keys": list(THUMB_FLEXION_TIP_ERGONOMICS_KEYS),
        "command": command,
        "samples": {
            "thumb_natural_open": {
                "root_value": _ergonomics_values(open_sample, (THUMB_FLEXION_ROOT_ERGONOMICS_KEY,))[0],
                "tip_value": _ergonomics_group_values(open_sample, (THUMB_FLEXION_TIP_ERGONOMICS_KEYS,))[0],
                "ergonomics": open_sample,
            },
            "thumb_pinky_root_touch": {
                "root_value": _ergonomics_values(touch_sample, (THUMB_FLEXION_ROOT_ERGONOMICS_KEY,))[0],
                "tip_value": _ergonomics_group_values(touch_sample, (THUMB_FLEXION_TIP_ERGONOMICS_KEYS,))[0],
                "ergonomics": touch_sample,
            },
        },
    }


def _build_fingertip_contact_calibration(
    samples: dict[str, dict[str, list[float]]],
    output_path: str,
    *,
    min_hold_sec: float,
    release_hold_sec: float,
    candidate_gap_ratio: float,
    takeover_start_progress: float,
    activation_rise_sec: float,
    activation_release_sec: float,
    distance_filter_alpha: float,
    command_slew_per_cycle: int,
    phase_switch_sec: float,
    robot_commands: dict[str, list[int]],
    max_command_delta: int,
) -> dict[str, Any]:
    open_samples = samples["natural_open"]
    contacts: dict[str, Any] = {}
    for finger in FINGERTIP_CONTACT_FINGERS:
        contact_samples = samples[f"thumb_{finger}_tip_touch"][finger]
        open_values = open_samples[finger]
        if not contact_samples or not open_values:
            raise RuntimeError(f"no fingertip distance samples for {finger}")
        contact_median = _percentile(contact_samples, 0.50)
        contact_p95 = _percentile(contact_samples, 0.95)
        open_p05 = _percentile(open_values, 0.05)
        if open_p05 <= contact_p95:
            raise RuntimeError(
                f"{finger} contact and natural-open raw distances overlap; recapture with a clearer open hand"
            )
        contacts[finger] = {
            "human": {
                "natural_open_distance_p05_ratio": round(open_p05, 6),
                "contact_distance_median_ratio": round(contact_median, 6),
                "contact_distance_p95_ratio": round(contact_p95, 6),
            },
            "robot": {
                "override_slots": list(_fingertip_contact_slots(finger)),
                "max_command_delta": int(max_command_delta),
                "contact_command": robot_commands[finger],
            },
        }
    return {
        "schema": "manus_l20.fingertip_contact_semantics.v2",
        "source": {
            "kind": "raw_skeleton_tip_distance",
            "numerator": "Thumb.TIP to Finger.TIP",
            "denominator": "Index.MCP to Pinky.MCP",
            "unit": "palm_width_ratio",
        },
        "runtime": {
            "enabled": True,
            "min_hold_sec": round(float(min_hold_sec), 4),
            "release_hold_sec": round(float(release_hold_sec), 4),
            "candidate_gap_ratio": round(float(candidate_gap_ratio), 6),
            "takeover_start_progress": round(float(takeover_start_progress), 4),
            "release_start_progress_delta": 0.06,
            "reclose_start_progress_delta": 0.005,
            "activation_rise_sec": round(float(activation_rise_sec), 4),
            "activation_release_sec": round(float(activation_release_sec), 4),
            "distance_filter_alpha": round(float(distance_filter_alpha), 4),
            "command_slew_per_cycle": int(command_slew_per_cycle),
            "phase_switch_sec": round(float(phase_switch_sec), 4),
        },
        "contacts": contacts,
    }


def _fingertip_contact_slots(finger: str) -> tuple[int, ...]:
    return FINGERTIP_CONTACT_SLOTS[finger]


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("cannot calculate percentile for an empty sample")
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _existing_fingertip_contact_command(path: str, finger: str, fallback: list[int]) -> list[int]:
    try:
        with Path(path).expanduser().open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return list(fallback)
    contacts = data.get("contacts") if isinstance(data, dict) else None
    entry = contacts.get(finger) if isinstance(contacts, dict) else None
    robot = entry.get("robot") if isinstance(entry, dict) else None
    if not isinstance(robot, dict):
        return list(fallback)
    return _command_parameter(robot.get("contact_command"), fallback)


def _spin_capture_node(node: Node) -> threading.Thread:
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    return spin_thread


def _shutdown_capture_node(node: Node, spin_thread: threading.Thread) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)


def _save_yaml(payload: dict[str, Any], output: str) -> Path:
    output_path = Path(output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        handle.write(_dump_yaml_with_l20_command_comments(payload))
    return output_path


def _dump_yaml_with_l20_command_comments(payload: dict[str, Any]) -> str:
    text = yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)
    lines = text.splitlines()
    output: list[str] = []
    index = 0
    while index < len(lines):
        line = lines[index]
        output.append(line)
        key = line.strip()[:-1] if line.strip().endswith(":") else ""
        if _is_l20_command_key(key) and _next_block_is_20_item_list(lines, index + 1, _indent_of(line)):
            for slot, comment in enumerate(L20_COMMAND_SLOT_COMMENTS):
                index += 1
                output.append(_append_slot_comment(lines[index], comment))
        index += 1
    return "\n".join(output) + "\n"


def _is_l20_command_key(key: str) -> bool:
    return key.endswith("_command") or key in {"robot_open_command", "robot_touch_command"}


def _next_block_is_20_item_list(lines: list[str], start_index: int, parent_indent: int) -> bool:
    count = 0
    for line in lines[start_index:]:
        if not line.strip():
            continue
        indent = _indent_of(line)
        if line.lstrip().startswith("- "):
            if indent < parent_indent:
                break
            count += 1
            if count > len(L20_COMMAND_SLOT_COMMENTS):
                return False
            continue
        if indent <= parent_indent:
            break
        break
    return count == len(L20_COMMAND_SLOT_COMMENTS)


def _append_slot_comment(line: str, comment: str) -> str:
    value = line.split("#", 1)[0].rstrip()
    return f"{value:<10} # {comment}"


def _indent_of(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def _validate_fingertip_contact_values(
    *,
    min_hold_sec: float,
    release_hold_sec: float,
    candidate_gap_ratio: float,
    takeover_start_progress: float,
    activation_rise_sec: float,
    activation_release_sec: float,
    distance_filter_alpha: float,
    command_slew_per_cycle: int,
    phase_switch_sec: float,
    max_command_delta: int,
) -> None:
    nonnegative = {
        "min_hold_sec": min_hold_sec,
        "release_hold_sec": release_hold_sec,
        "candidate_gap_ratio": candidate_gap_ratio,
        "activation_rise_sec": activation_rise_sec,
        "activation_release_sec": activation_release_sec,
        "distance_filter_alpha": distance_filter_alpha,
        "command_slew_per_cycle": command_slew_per_cycle,
        "phase_switch_sec": phase_switch_sec,
        "max_command_delta": max_command_delta,
    }
    invalid = [
        name
        for name, value in nonnegative.items()
        if not math.isfinite(float(value)) or float(value) < 0.0
    ]
    if invalid:
        raise RuntimeError(f"fingertip contact arguments must be nonnegative: {', '.join(invalid)}")
    if not math.isfinite(takeover_start_progress) or not 0.0 <= takeover_start_progress < 1.0:
        raise RuntimeError("takeover-start-progress must be in [0.0, 1.0)")
    if not math.isfinite(float(distance_filter_alpha)) or not 0.0 <= float(distance_filter_alpha) <= 1.0:
        raise RuntimeError("distance-filter-alpha must be in [0.0, 1.0]")


def _config_output_path(hand: str, filename: str, override: str) -> str:
    if str(override).strip():
        return str(Path(override).expanduser())
    root = _default_workspace_root()
    return str(root / "src" / "manus_l20_retarget" / "config" / filename.format(hand=hand))


class FullCalibrationSession:
    """Capture the five MANUS poses required by the Qt calibration workflow."""

    def __init__(
        self,
        *,
        hand: str,
        glove_topic: str,
        duration: float = 2.0,
        transform: str = "right_glove_to_right_retarget",
        segment_start: int = 2,
        segment_end: int = 3,
        ergonomics_keys: str = "",
        finger_flexion_ergonomics_output: str = "",
        finger_yaw_output: str = "",
        thumb_flexion_ergonomics_output: str = "",
        thumb_frame_output: str = "",
        robot_open_command: Any = "",
        robot_touch_command: Any = "",
    ) -> None:
        self.hand = str(hand).strip().lower()
        if self.hand not in {"right", "left"}:
            raise ValueError("hand must be 'right' or 'left'")
        self.glove_topic = str(glove_topic)
        self.duration = max(0.01, float(duration))
        self.transform = str(transform)
        if self.hand == "left" and self.transform == "right_glove_to_right_retarget":
            self.transform = "left_glove_to_right_retarget"
        self.segment_start = int(segment_start)
        self.segment_end = int(segment_end)
        self.ergonomics_keys = _parse_key_list(ergonomics_keys)
        self.robot_open_command = robot_open_command
        self.robot_touch_command = robot_touch_command
        self.finger_flexion_output = _config_output_path(
            self.hand,
            "finger_flexion_ergonomics_{hand}_calibration.yaml",
            finger_flexion_ergonomics_output,
        )
        self.yaw_output = _config_output_path(
            self.hand,
            "finger_yaw_ergonomics_{hand}_calibration.yaml",
            finger_yaw_output,
        )
        self.thumb_flexion_output = _config_output_path(
            self.hand,
            "thumb_{hand}_flexion_ergonomics_mapping.yaml",
            thumb_flexion_ergonomics_output,
        )
        self.thumb_frame_output = _config_output_path(
            self.hand,
            "thumb_segment_frame_{hand}.yaml",
            thumb_frame_output,
        )
        self.samples: dict[str, dict[str, Any]] = {}
        self._node: FullCalibrationCapture | None = None
        self._spin_thread: threading.Thread | None = None

    def start(self) -> None:
        if self._node is not None:
            return
        if rclpy.ok():
            raise RuntimeError("another ROS2 calibration session is already active")
        rclpy.init()
        try:
            self._node = FullCalibrationCapture(
                self.glove_topic,
                self.transform,
                self.segment_start,
                self.segment_end,
            )
            self._spin_thread = _spin_capture_node(self._node)
        except Exception:
            if rclpy.ok():
                rclpy.shutdown()
            self._node = None
            self._spin_thread = None
            raise

    def capture_pose(self, label: str) -> dict[str, Any]:
        if label not in dict(FULL_CALIBRATION_POSES):
            raise ValueError(f"unknown full-calibration pose: {label}")
        self.start()
        assert self._node is not None
        sample = self._node.capture(self.duration)
        self.samples[label] = sample
        return sample

    def save(self) -> list[Path]:
        missing = [label for label, _ in FULL_CALIBRATION_POSES if label not in self.samples]
        if missing:
            raise RuntimeError(f"calibration poses have not been captured: {', '.join(missing)}")

        yaw_calibration = _build_finger_yaw_ergonomics_calibration(
            {
                "natural_open": self.samples["natural_open"]["yaw"],
                "finger_close": self.samples["finger_close"]["yaw"],
                "finger_spread": self.samples["finger_spread"]["yaw"],
            },
            self.yaw_output,
            self.ergonomics_keys,
        )
        finger_flexion_calibration = _build_finger_flexion_ergonomics_calibration(
            {
                "natural_open": self.samples["natural_open"]["yaw"],
                "four_finger_fist": self.samples["four_finger_fist"]["yaw"],
            },
            self.finger_flexion_output,
        )
        thumb_flexion_calibration = _build_thumb_flexion_ergonomics_calibration(
            {
                "thumb_natural_open": self.samples["natural_open"]["yaw"],
                "thumb_pinky_root_touch": self.samples["thumb_pinky_root_touch"]["yaw"],
            },
            self.thumb_flexion_output,
        )

        parsed_robot_open = _command_parameter(self.robot_open_command, [])
        robot_open_command = (
            _existing_top_level_command(self.thumb_frame_output, "robot_open_command", parsed_robot_open)
            if parsed_robot_open
            else _existing_top_level_command(self.thumb_frame_output, "robot_open_command", STANDARD_OPEN_COMMAND)
        )
        parsed_robot_touch = _command_parameter(self.robot_touch_command, [])
        if parsed_robot_touch:
            robot_touch_command = parsed_robot_touch
        else:
            touch_fallback = _command_parameter(
                thumb_flexion_calibration["command"].get("thumb_pinky_root_touch_command"),
                robot_open_command,
            )
            robot_touch_command = _existing_top_level_command(
                self.thumb_frame_output,
                "robot_touch_command",
                touch_fallback,
            )
        thumb_frame_calibration = {
            "schema": "manus_l20.thumb_segment_frame.v1",
            "segment_start": self.segment_start,
            "segment_end": self.segment_end,
            "manus_open_vector": self.samples["natural_open"]["thumb_segment_vector"],
            "manus_touch_vector": self.samples["thumb_pinky_root_touch"]["thumb_segment_vector"],
            "robot_open_command": robot_open_command,
            "robot_touch_command": robot_touch_command,
        }
        return [
            _save_yaml(finger_flexion_calibration, self.finger_flexion_output),
            _save_yaml(yaw_calibration, self.yaw_output),
            _save_yaml(thumb_flexion_calibration, self.thumb_flexion_output),
            _save_yaml(thumb_frame_calibration, self.thumb_frame_output),
        ]

    def close(self) -> None:
        if self._node is None or self._spin_thread is None:
            return
        _shutdown_capture_node(self._node, self._spin_thread)
        self._node = None
        self._spin_thread = None


class FingertipContactCalibrationSession:
    """Capture raw-skeleton contact semantics without commanding L20."""

    def __init__(
        self,
        *,
        hand: str,
        glove_topic: str,
        duration: float = 2.0,
        transform: str = "right_glove_to_right_retarget",
        output: str = "",
        min_hold_sec: float = 0.10,
        release_hold_sec: float = 0.05,
        candidate_gap_ratio: float = 0.02,
        takeover_start_progress: float = 0.35,
        activation_rise_sec: float = 0.04,
        activation_release_sec: float = 0.25,
        distance_filter_alpha: float = 0.85,
        command_slew_per_cycle: int = 255,
        phase_switch_sec: float = 0.25,
        max_command_delta: int = 255,
        robot_commands: dict[str, Any] | None = None,
    ) -> None:
        self.hand = str(hand).strip().lower()
        if self.hand not in {"right", "left"}:
            raise ValueError("hand must be 'right' or 'left'")
        self.glove_topic = str(glove_topic)
        self.duration = max(0.01, float(duration))
        self.transform = str(transform)
        if self.hand == "left" and self.transform == "right_glove_to_right_retarget":
            self.transform = "left_glove_to_right_retarget"
        self.output = _config_output_path(self.hand, "fingertip_contact_semantics_{hand}.yaml", output)
        self.min_hold_sec = float(min_hold_sec)
        self.release_hold_sec = float(release_hold_sec)
        self.candidate_gap_ratio = float(candidate_gap_ratio)
        self.takeover_start_progress = float(takeover_start_progress)
        self.activation_rise_sec = float(activation_rise_sec)
        self.activation_release_sec = float(activation_release_sec)
        self.distance_filter_alpha = float(distance_filter_alpha)
        self.command_slew_per_cycle = int(command_slew_per_cycle)
        self.phase_switch_sec = float(phase_switch_sec)
        self.max_command_delta = int(max_command_delta)
        _validate_fingertip_contact_values(
            min_hold_sec=self.min_hold_sec,
            release_hold_sec=self.release_hold_sec,
            candidate_gap_ratio=self.candidate_gap_ratio,
            takeover_start_progress=self.takeover_start_progress,
            activation_rise_sec=self.activation_rise_sec,
            activation_release_sec=self.activation_release_sec,
            distance_filter_alpha=self.distance_filter_alpha,
            command_slew_per_cycle=self.command_slew_per_cycle,
            phase_switch_sec=self.phase_switch_sec,
            max_command_delta=self.max_command_delta,
        )
        provided_commands = robot_commands or {}
        self.robot_commands = {
            finger: (
                _command_parameter(provided_commands.get(finger), [])
                or _existing_fingertip_contact_command(self.output, finger, STANDARD_OPEN_COMMAND)
            )
            for finger in FINGERTIP_CONTACT_FINGERS
        }
        self.samples: dict[str, dict[str, list[float]]] = {}
        self._node: FingertipContactCapture | None = None
        self._spin_thread: threading.Thread | None = None

    def start(self) -> None:
        if self._node is not None:
            return
        if rclpy.ok():
            raise RuntimeError("another ROS2 calibration session is already active")
        rclpy.init()
        try:
            self._node = FingertipContactCapture(self.glove_topic, self.transform)
            self._spin_thread = _spin_capture_node(self._node)
        except Exception:
            if rclpy.ok():
                rclpy.shutdown()
            self._node = None
            self._spin_thread = None
            raise

    def capture_pose(self, label: str) -> dict[str, list[float]]:
        if label not in dict(FINGERTIP_CONTACT_POSES):
            raise ValueError(f"unknown fingertip-contact pose: {label}")
        self.start()
        assert self._node is not None
        sample = self._node.capture(self.duration)
        self.samples[label] = sample
        return sample

    def save(self) -> Path:
        missing = [label for label, _ in FINGERTIP_CONTACT_POSES if label not in self.samples]
        if missing:
            raise RuntimeError(f"fingertip-contact poses have not been captured: {', '.join(missing)}")
        calibration = _build_fingertip_contact_calibration(
            self.samples,
            self.output,
            min_hold_sec=self.min_hold_sec,
            release_hold_sec=self.release_hold_sec,
            candidate_gap_ratio=self.candidate_gap_ratio,
            takeover_start_progress=self.takeover_start_progress,
            activation_rise_sec=self.activation_rise_sec,
            activation_release_sec=self.activation_release_sec,
            distance_filter_alpha=self.distance_filter_alpha,
            command_slew_per_cycle=self.command_slew_per_cycle,
            phase_switch_sec=self.phase_switch_sec,
            robot_commands=self.robot_commands,
            max_command_delta=self.max_command_delta,
        )
        return _save_yaml(calibration, self.output)

    def close(self) -> None:
        if self._node is None or self._spin_thread is None:
            return
        _shutdown_capture_node(self._node, self._spin_thread)
        self._node = None
        self._spin_thread = None


def _parse_key_list(value: str) -> list[str] | None:
    keys = [item.strip() for item in str(value).split(",") if item.strip()]
    if not keys:
        return None
    if len(keys) != 4:
        raise RuntimeError("ergonomics keys must contain exactly 4 values: index,middle,ring,pinky")
    return keys
