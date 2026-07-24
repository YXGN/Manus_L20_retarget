from __future__ import annotations

import argparse
import statistics
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node

from .manus_landmarks import (
    _finger_joint_orientation_yaw_rad,
    _finger_mcp_orientation_yaw_rad,
    _finger_yaw_rad,
    manus_raw_nodes_to_mediapipe_landmarks,
)
from .manus_l20_retarget_node import (
    FINGER_LANDMARKS,
    STANDARD_OPEN_COMMAND,
    _command_parameter,
    _default_workspace_root,
    _manus_thumb_local_point_at,
)
from .retarget_pipeline import _directed_finger_flexion_rad, _joint_flexion_rad


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
POSITION_SOURCES = ("pip", "dip", "tip")
ORIENTATION_SOURCES = ("mcp_orientation", "pip_orientation", "ip_orientation", "dip_orientation")
ORIENTATION_AND_POSITION_SOURCES = (*POSITION_SOURCES, *ORIENTATION_SOURCES)
FINGER_NAMES = ("index", "middle", "ring", "pinky")
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


class FlexionCalibrationCapture(Node):
    def __init__(self, glove_topic: str, transform: str, wrist_mode: str, distal_mode: str) -> None:
        super().__init__("manus_l20_flexion_calibration_capture")
        self._transform = transform
        self._wrist_mode = wrist_mode
        self._distal_mode = distal_mode
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[dict[str, list[float]]] = []
        self.create_subscription(ManusGlove, glove_topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=self._transform,
            wrist_mode=self._wrist_mode,
            distal_mode=self._distal_mode,
        )
        sample = _flexion_angles(landmarks)
        with self._lock:
            if self._collecting:
                self._samples.append(sample)

    def capture(self, duration_sec: float) -> dict[str, list[float]]:
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
            "root_rad": _mean_vector([sample["root_rad"] for sample in samples]),
            "tip_rad": _mean_vector([sample["tip_rad"] for sample in samples]),
            "root_signed_rad": _mean_vector([sample["root_signed_rad"] for sample in samples]),
            "tip_signed_rad": _mean_vector([sample["tip_signed_rad"] for sample in samples]),
        }


class FingerYawCalibrationCapture(Node):
    def __init__(
        self,
        glove_topic: str,
        transform: str,
        wrist_mode: str,
        distal_mode: str,
        yaw_source: str,
    ) -> None:
        super().__init__("manus_l20_finger_yaw_calibration_capture")
        self._transform = transform
        self._wrist_mode = wrist_mode
        self._distal_mode = distal_mode
        self._yaw_source = yaw_source
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[dict[str, list[float]]] = []
        self.create_subscription(ManusGlove, glove_topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=self._transform,
            wrist_mode=self._wrist_mode,
            distal_mode=self._distal_mode,
        )
        yaw_rad_by_source = {
            source: [float(value) for value in _finger_yaw_rad(landmarks, source=source)]
            for source in POSITION_SOURCES
        }
        yaw_rad_by_source["mcp_orientation"] = [
            float(value) for value in _finger_mcp_orientation_yaw_rad(msg.raw_nodes)
        ]
        for source, joint_type in (
            ("pip_orientation", "PIP"),
            ("ip_orientation", "IP"),
            ("dip_orientation", "DIP"),
        ):
            yaw_rad_by_source[source] = [
                float(value) for value in _finger_joint_orientation_yaw_rad(msg.raw_nodes, joint_type=joint_type)
            ]
        with self._lock:
            if self._collecting:
                self._samples.append(yaw_rad_by_source)

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
        all_yaw_rad = {
            source: _mean_vector([sample[source] for sample in samples])
            for source in ORIENTATION_AND_POSITION_SOURCES
        }
        selected_source = self._yaw_source if self._yaw_source in all_yaw_rad else "pip"
        return {
            "yaw_rad": all_yaw_rad[selected_source],
            "all_yaw_rad": all_yaw_rad,
        }


class FingerYawErgonomicsCalibrationCapture(Node):
    def __init__(self, glove_topic: str) -> None:
        super().__init__("manus_l20_finger_yaw_ergonomics_calibration_capture")
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[dict[str, float]] = []
        self.create_subscription(ManusGlove, glove_topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        ergonomics = {str(entry.type): float(entry.value) for entry in msg.ergonomics if entry.type}
        with self._lock:
            if self._collecting:
                self._samples.append(ergonomics)

    def capture(self, duration_sec: float) -> dict[str, float]:
        with self._lock:
            self._samples = []
            self._collecting = True
        time.sleep(duration_sec)
        with self._lock:
            self._collecting = False
            samples = list(self._samples)
        if not samples:
            raise RuntimeError("no MANUS glove samples captured")
        keys = sorted({key for sample in samples for key in sample})
        return {
            key: round(float(statistics.fmean(sample[key] for sample in samples if key in sample)), 6)
            for key in keys
            if any(key in sample for sample in samples)
        }


class ThumbFlexionMappingCapture(Node):
    def __init__(self, glove_topic: str, transform: str, wrist_mode: str, distal_mode: str) -> None:
        super().__init__("manus_l20_thumb_flexion_mapping_capture")
        self._transform = transform
        self._wrist_mode = wrist_mode
        self._distal_mode = distal_mode
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[dict[str, float]] = []
        self.create_subscription(ManusGlove, glove_topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=self._transform,
            wrist_mode=self._wrist_mode,
            distal_mode=self._distal_mode,
        )
        thumb_mcp, thumb_pip, thumb_dip, thumb_tip = FINGER_LANDMARKS[0]
        sample = {
            "root_rad": _joint_flexion_rad(landmarks[thumb_mcp], landmarks[thumb_pip], landmarks[thumb_dip]),
            "tip_rad": _joint_flexion_rad(landmarks[thumb_pip], landmarks[thumb_dip], landmarks[thumb_tip]),
            "root_signed_rad": _directed_finger_flexion_rad(landmarks, 0, root=True),
            "tip_signed_rad": _directed_finger_flexion_rad(landmarks, 0, root=False),
        }
        with self._lock:
            if self._collecting:
                self._samples.append(sample)

    def capture(self, duration_sec: float) -> dict[str, float]:
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
            "root_rad": round(float(statistics.fmean(sample["root_rad"] for sample in samples)), 6),
            "tip_rad": round(float(statistics.fmean(sample["tip_rad"] for sample in samples)), 6),
            "root_signed_rad": round(float(statistics.fmean(sample["root_signed_rad"] for sample in samples)), 6),
            "tip_signed_rad": round(float(statistics.fmean(sample["tip_signed_rad"] for sample in samples)), 6),
        }


class ThumbSegmentVectorCapture(Node):
    def __init__(
        self,
        node_name: str,
        glove_topic: str,
        transform: str,
        wrist_mode: str,
        distal_mode: str,
        segment_start: int,
        segment_end: int,
    ) -> None:
        super().__init__(node_name)
        self._transform = transform
        self._wrist_mode = wrist_mode
        self._distal_mode = distal_mode
        self._segment_start = segment_start
        self._segment_end = segment_end
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[list[float]] = []
        self.create_subscription(ManusGlove, glove_topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=self._transform,
            wrist_mode=self._wrist_mode,
            distal_mode=self._distal_mode,
        )
        start = _manus_thumb_local_point_at(landmarks, self._segment_start)
        end = _manus_thumb_local_point_at(landmarks, self._segment_end)
        vector = end if self._segment_start == self._segment_end else end - start
        with self._lock:
            if self._collecting:
                self._samples.append([float(value) for value in vector])

    def capture(self, duration_sec: float) -> list[float]:
        with self._lock:
            self._samples = []
            self._collecting = True
        time.sleep(duration_sec)
        with self._lock:
            self._collecting = False
            samples = list(self._samples)
        if not samples:
            raise RuntimeError("no MANUS glove samples captured")
        return _mean_vector(samples)


class FullCalibrationCapture(Node):
    def __init__(
        self,
        glove_topic: str,
        transform: str,
        wrist_mode: str,
        distal_mode: str,
        segment_start: int,
        segment_end: int,
    ) -> None:
        super().__init__("manus_l20_full_calibration_capture")
        self._transform = transform
        self._wrist_mode = wrist_mode
        self._distal_mode = distal_mode
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
            wrist_mode=self._wrist_mode,
            distal_mode=self._distal_mode,
        )
        thumb_mcp, thumb_pip, thumb_dip, thumb_tip = FINGER_LANDMARKS[0]
        start = _manus_thumb_local_point_at(landmarks, self._segment_start)
        end = _manus_thumb_local_point_at(landmarks, self._segment_end)
        vector = end if self._segment_start == self._segment_end else end - start
        yaw_rad_by_source = {
            source: [float(value) for value in _finger_yaw_rad(landmarks, source=source)]
            for source in POSITION_SOURCES
        }
        yaw_rad_by_source["mcp_orientation"] = [
            float(value) for value in _finger_mcp_orientation_yaw_rad(msg.raw_nodes)
        ]
        for source, joint_type in (
            ("pip_orientation", "PIP"),
            ("ip_orientation", "IP"),
            ("dip_orientation", "DIP"),
        ):
            yaw_rad_by_source[source] = [
                float(value) for value in _finger_joint_orientation_yaw_rad(msg.raw_nodes, joint_type=joint_type)
            ]
        sample = {
            "flexion": _flexion_angles(landmarks),
            "thumb_flexion": {
                "root_rad": _joint_flexion_rad(landmarks[thumb_mcp], landmarks[thumb_pip], landmarks[thumb_dip]),
                "tip_rad": _joint_flexion_rad(landmarks[thumb_pip], landmarks[thumb_dip], landmarks[thumb_tip]),
                "root_signed_rad": _directed_finger_flexion_rad(landmarks, 0, root=True),
                "tip_signed_rad": _directed_finger_flexion_rad(landmarks, 0, root=False),
            },
            "thumb_segment_vector": [float(value) for value in vector],
            "yaw": yaw_rad_by_source,
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
            "flexion": {
                "root_rad": _mean_vector([sample["flexion"]["root_rad"] for sample in samples]),
                "tip_rad": _mean_vector([sample["flexion"]["tip_rad"] for sample in samples]),
                "root_signed_rad": _mean_vector([sample["flexion"]["root_signed_rad"] for sample in samples]),
                "tip_signed_rad": _mean_vector([sample["flexion"]["tip_signed_rad"] for sample in samples]),
            },
            "thumb_flexion": {
                "root_rad": round(float(statistics.fmean(sample["thumb_flexion"]["root_rad"] for sample in samples)), 6),
                "tip_rad": round(float(statistics.fmean(sample["thumb_flexion"]["tip_rad"] for sample in samples)), 6),
                "root_signed_rad": round(float(statistics.fmean(sample["thumb_flexion"]["root_signed_rad"] for sample in samples)), 6),
                "tip_signed_rad": round(float(statistics.fmean(sample["thumb_flexion"]["tip_signed_rad"] for sample in samples)), 6),
            },
            "thumb_segment_vector": _mean_vector([sample["thumb_segment_vector"] for sample in samples]),
            "yaw": {
                "yaw_rad": [],
                "all_yaw_rad": {
                    source: _mean_vector([sample["yaw"][source] for sample in samples])
                    for source in ORIENTATION_AND_POSITION_SOURCES
                },
            },
        }


def _flexion_angles(landmarks: Any) -> dict[str, list[float]]:
    root_angles: list[float] = []
    tip_angles: list[float] = []
    root_signed_angles: list[float] = []
    tip_signed_angles: list[float] = []
    for finger_index, (mcp, pip, dip, tip) in enumerate(FINGER_LANDMARKS):
        root_angles.append(_joint_flexion_rad(landmarks[mcp], landmarks[pip], landmarks[dip]))
        tip_angles.append(_joint_flexion_rad(landmarks[pip], landmarks[dip], landmarks[tip]))
        root_signed_angles.append(_directed_finger_flexion_rad(landmarks, finger_index, root=True))
        tip_signed_angles.append(_directed_finger_flexion_rad(landmarks, finger_index, root=False))
    return {
        "root_rad": root_angles,
        "tip_rad": tip_angles,
        "root_signed_rad": root_signed_angles,
        "tip_signed_rad": tip_signed_angles,
    }


def _mean_vector(samples: list[list[float]]) -> list[float]:
    length = min(len(sample) for sample in samples)
    return [
        round(float(statistics.fmean(sample[index] for sample in samples)), 6)
        for index in range(length)
    ]


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


def _direction_signs(open_values: list[float] | None, closed_values: list[float] | None, length: int) -> list[float]:
    if open_values is None or closed_values is None:
        return [1.0] * length
    signs: list[float] = []
    for index in range(length):
        try:
            delta = float(closed_values[index]) - float(open_values[index])
        except (IndexError, TypeError, ValueError):
            delta = 0.0
        signs.append(1.0 if delta >= 0.0 else -1.0)
    return signs


def _apply_direction_signs(values: list[float] | None, signs: list[float], fallback: list[float]) -> list[float]:
    if values is None:
        return list(fallback)
    output: list[float] = []
    for index, sign in enumerate(signs):
        try:
            output.append(round(max(0.0, float(values[index]) * float(sign)), 6))
        except (IndexError, TypeError, ValueError):
            output.append(round(float(fallback[index]), 6))
    return output


def _build_flexion_calibration(samples: dict[str, dict[str, list[float]]], output_path: str) -> dict[str, Any]:
    open_sample = samples["open"]
    four_finger_sample = samples["four_finger_fist"]
    root_signs = _direction_signs(open_sample.get("root_signed_rad"), four_finger_sample.get("root_signed_rad"), 5)
    tip_signs = _direction_signs(open_sample.get("tip_signed_rad"), four_finger_sample.get("tip_signed_rad"), 5)
    root_open = _apply_direction_signs(open_sample.get("root_signed_rad"), root_signs, open_sample["root_rad"])
    tip_open = _apply_direction_signs(open_sample.get("tip_signed_rad"), tip_signs, open_sample["tip_rad"])
    root_fist = _apply_direction_signs(four_finger_sample.get("root_signed_rad"), root_signs, four_finger_sample["root_rad"])
    tip_fist = _apply_direction_signs(four_finger_sample.get("tip_signed_rad"), tip_signs, four_finger_sample["tip_rad"])
    root_closed = [root_open[0], *root_fist[1:]]
    tip_closed = [tip_open[0], *tip_fist[1:]]
    command = _existing_command(output_path) or {
        "open_command": DEFAULT_L20_OPEN_COMMAND,
        "four_finger_closed_command": DEFAULT_L20_FOUR_FINGER_CLOSED_COMMAND,
    }
    return {
        "schema": "manus_l20.flexion_calibration.v1",
        "finger_order": ["thumb", "index", "middle", "ring", "pinky"],
        "root_flexion_open_rad": root_open,
        "root_flexion_closed_rad": root_closed,
        "tip_flexion_open_rad": tip_open,
        "tip_flexion_closed_rad": tip_closed,
        "root_flexion_direction_sign": root_signs,
        "tip_flexion_direction_sign": tip_signs,
        "command": command,
        "samples": samples,
    }


def _build_finger_yaw_calibration(samples: dict[str, dict[str, Any]], yaw_source: str, output_path: str) -> dict[str, Any]:
    selected_source = yaw_source
    if yaw_source in ("auto", "auto_orientation"):
        selected_source = _select_best_source(samples, mode=yaw_source)
    selected_samples = {
        label: {
            "yaw_rad": sample["all_yaw_rad"][selected_source],
            "all_yaw_rad": sample["all_yaw_rad"],
        }
        for label, sample in samples.items()
    }
    command = _existing_command(output_path) or {
        "natural_open_command": DEFAULT_L20_OPEN_COMMAND,
        "finger_close_command": DEFAULT_L20_FINGER_CLOSE_COMMAND,
        "finger_spread_command": DEFAULT_L20_FINGER_SPREAD_COMMAND,
    }
    return {
        "schema": "manus_l20.finger_yaw_calibration.v1",
        "finger_order": ["index", "middle", "ring", "pinky"],
        "source": selected_source,
        "source_mode": yaw_source,
        "source_ranges": _source_ranges(samples),
        "command": command,
        "samples": selected_samples,
    }


def _source_ranges(samples: dict[str, dict[str, Any]]) -> dict[str, list[float]]:
    ranges: dict[str, list[float]] = {}
    open_sample = samples["natural_open"]["all_yaw_rad"]
    close_sample = samples["finger_close"]["all_yaw_rad"]
    spread_sample = samples["finger_spread"]["all_yaw_rad"]
    for source in ORIENTATION_AND_POSITION_SOURCES:
        ranges[source] = [
            round(
                max(
                    abs(float(close_sample[source][index]) - float(open_sample[source][index])),
                    abs(float(spread_sample[source][index]) - float(open_sample[source][index])),
                ),
                6,
            )
            for index in range(4)
        ]
    return ranges


def _select_best_source(samples: dict[str, dict[str, Any]], *, mode: str) -> str:
    ranges = _source_ranges(samples)
    candidate_sources = ORIENTATION_SOURCES if mode == "auto_orientation" else tuple(ranges)
    totals = {
        source: sum(values)
        for source, values in ranges.items()
        if source in candidate_sources
    }
    return max(totals, key=totals.get)


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


def _build_thumb_flexion_calibration(samples: dict[str, dict[str, float]], output_path: str) -> dict[str, Any]:
    open_sample = samples["thumb_natural_open"]
    touch_sample = samples["thumb_pinky_root_touch"]
    root_sign = _direction_signs(
        [open_sample.get("root_signed_rad", open_sample["root_rad"])],
        [touch_sample.get("root_signed_rad", touch_sample["root_rad"])],
        1,
    )[0]
    tip_sign = _direction_signs(
        [open_sample.get("tip_signed_rad", open_sample["tip_rad"])],
        [touch_sample.get("tip_signed_rad", touch_sample["tip_rad"])],
        1,
    )[0]
    samples = {
        "thumb_pinky_root_touch": {
            **touch_sample,
            "root_rad": round(max(0.0, float(touch_sample.get("root_signed_rad", touch_sample["root_rad"])) * root_sign), 6),
            "tip_rad": round(max(0.0, float(touch_sample.get("tip_signed_rad", touch_sample["tip_rad"])) * tip_sign), 6),
        },
        "thumb_natural_open": {
            **open_sample,
            "root_rad": round(max(0.0, float(open_sample.get("root_signed_rad", open_sample["root_rad"])) * root_sign), 6),
            "tip_rad": round(max(0.0, float(open_sample.get("tip_signed_rad", open_sample["tip_rad"])) * tip_sign), 6),
        },
    }
    command = _existing_command(output_path) or {
        "thumb_natural_open_command": DEFAULT_THUMB_NATURAL_OPEN_COMMAND,
        "thumb_pinky_root_touch_command": None,
    }
    return {
        "schema": "manus_l20.thumb_flexion_mapping.v1",
        "finger": "thumb",
        "root_flexion_direction_sign": root_sign,
        "tip_flexion_direction_sign": tip_sign,
        "command": command,
        "samples": samples,
    }


def _load_command(path_value: str, key: str, fallback: list[int]) -> list[int]:
    path_text = str(path_value).strip()
    if not path_text:
        return list(fallback)
    path = Path(path_text).expanduser()
    if not path.exists():
        return list(fallback)
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return list(fallback)
    command_data = data.get("command")
    if not isinstance(command_data, dict):
        return list(fallback)
    return _command_parameter(command_data.get(key), fallback)


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


def run_flexion(parsed: argparse.Namespace) -> None:
    rclpy.init()
    node = FlexionCalibrationCapture(parsed.glove_topic, parsed.transform, parsed.wrist_mode, parsed.distal_mode)
    spin_thread = _spin_capture_node(node)
    labels = [
        ("open", "张开手掌，五指尽量完全伸直"),
        ("four_finger_fist", "四指完全弯曲握拳，拇指姿势不用管"),
    ]
    samples: dict[str, dict[str, list[float]]] = {}
    try:
        for label, prompt in labels:
            input(f"Set pose '{label}' ({prompt}), hold still, then press Enter...")
            print(f"Capturing '{label}' for {parsed.duration:.1f}s...")
            samples[label] = node.capture(parsed.duration)
            print(yaml.safe_dump({label: samples[label]}, sort_keys=False, allow_unicode=True))
        calibration = _build_flexion_calibration(samples, parsed.output)
        output_path = _save_yaml(calibration, parsed.output)
        print(f"Saved flexion calibration to {output_path}")
    finally:
        _shutdown_capture_node(node, spin_thread)


def run_finger_yaw(parsed: argparse.Namespace) -> None:
    rclpy.init()
    node = FingerYawCalibrationCapture(
        parsed.glove_topic,
        parsed.transform,
        parsed.wrist_mode,
        parsed.distal_mode,
        parsed.yaw_source,
    )
    spin_thread = _spin_capture_node(node)
    labels = [
        ("natural_open", "手指自然张开"),
        ("finger_close", "四指并拢"),
        ("finger_spread", "四指外展"),
    ]
    samples: dict[str, dict[str, Any]] = {}
    try:
        for label, prompt in labels:
            input(f"Set pose '{label}' ({prompt}), hold still, then press Enter...")
            print(f"Capturing '{label}' for {parsed.duration:.1f}s...")
            samples[label] = node.capture(parsed.duration)
            print(yaml.safe_dump({label: samples[label]}, sort_keys=False, allow_unicode=True))
        calibration = _build_finger_yaw_calibration(samples, parsed.yaw_source, parsed.output)
        output_path = _save_yaml(calibration, parsed.output)
        print(
            yaml.safe_dump(
                {
                    "selected_source": calibration["source"],
                    "source_ranges": calibration["source_ranges"],
                },
                sort_keys=False,
                allow_unicode=True,
            )
        )
        print(f"Saved finger yaw calibration to {output_path}")
    finally:
        _shutdown_capture_node(node, spin_thread)


def run_finger_yaw_ergonomics(parsed: argparse.Namespace) -> None:
    rclpy.init()
    node = FingerYawErgonomicsCalibrationCapture(parsed.glove_topic)
    spin_thread = _spin_capture_node(node)
    labels = [
        ("natural_open", "手指自然张开"),
        ("finger_close", "四指并拢"),
        ("finger_spread", "四指外展"),
    ]
    samples: dict[str, dict[str, float]] = {}
    try:
        for label, prompt in labels:
            input(f"Set pose '{label}' ({prompt}), hold still, then press Enter...")
            print(f"Capturing '{label}' ergonomics for {parsed.duration:.1f}s...")
            samples[label] = node.capture(parsed.duration)
            print(yaml.safe_dump({label: samples[label]}, sort_keys=False, allow_unicode=True))
        ergonomics_keys = _parse_key_list(parsed.ergonomics_keys)
        calibration = _build_finger_yaw_ergonomics_calibration(samples, parsed.output, ergonomics_keys)
        output_path = _save_yaml(calibration, parsed.output)
        print(
            yaml.safe_dump(
                {
                    "selected_source": calibration["source"],
                    "ergonomics_keys": calibration["ergonomics_keys"],
                    "ergonomics_source_ranges": calibration["ergonomics_source_ranges"],
                },
                sort_keys=False,
                allow_unicode=True,
            )
        )
        print(f"Saved ergonomics finger yaw calibration to {output_path}")
    finally:
        _shutdown_capture_node(node, spin_thread)


def run_thumb_flexion(parsed: argparse.Namespace) -> None:
    rclpy.init()
    node = ThumbFlexionMappingCapture(parsed.glove_topic, parsed.transform, parsed.wrist_mode, parsed.distal_mode)
    spin_thread = _spin_capture_node(node)
    labels = [
        ("thumb_pinky_root_touch", "大拇指触碰小拇指指根"),
        ("thumb_natural_open", "手指自然张开"),
    ]
    samples: dict[str, dict[str, float]] = {}
    try:
        for label, prompt in labels:
            input(f"Set pose '{label}' ({prompt}), hold still, then press Enter...")
            print(f"Capturing '{label}' for {parsed.duration:.1f}s...")
            samples[label] = node.capture(parsed.duration)
            print(yaml.safe_dump({label: samples[label]}, sort_keys=False, allow_unicode=True))
        calibration = _build_thumb_flexion_calibration(samples, parsed.output)
        output_path = _save_yaml(calibration, parsed.output)
        print(f"Saved thumb flexion mapping calibration to {output_path}")
    finally:
        _shutdown_capture_node(node, spin_thread)


def run_thumb_open_vector(parsed: argparse.Namespace) -> None:
    rclpy.init()
    node = ThumbSegmentVectorCapture(
        "manus_l20_thumb_segment_open_vector_capture",
        parsed.glove_topic,
        parsed.transform,
        parsed.wrist_mode,
        parsed.distal_mode,
        parsed.segment_start,
        parsed.segment_end,
    )
    spin_thread = _spin_capture_node(node)
    try:
        input("Set pose 'thumb_natural_open' (手自然张开，大拇指保持标准自然张开), hold still, then press Enter...")
        print(f"Capturing thumb segment open vector for {parsed.duration:.1f}s...")
        vector = node.capture(parsed.duration)
        output = {
            "schema": "manus_l20.thumb_segment_open_vector.v1",
            "segment_start": int(parsed.segment_start),
            "segment_end": int(parsed.segment_end),
            "manus_open_vector": vector,
        }
        output_path = _save_yaml(output, parsed.output)
        print(yaml.safe_dump(output, sort_keys=False, allow_unicode=True))
        print(f"Saved thumb segment open vector to {output_path}")
    finally:
        _shutdown_capture_node(node, spin_thread)


def run_thumb_frame(parsed: argparse.Namespace) -> None:
    robot_open_command = _command_parameter(parsed.robot_open_command, STANDARD_OPEN_COMMAND)
    robot_touch_command = _command_parameter(parsed.robot_touch_command, robot_open_command)
    if not str(parsed.robot_touch_command).strip():
        robot_touch_command = _load_command(
            parsed.thumb_flexion_mapping_path,
            "thumb_pinky_root_touch_command",
            robot_open_command,
        )

    rclpy.init()
    node = ThumbSegmentVectorCapture(
        "manus_l20_thumb_segment_frame_capture",
        parsed.glove_topic,
        parsed.transform,
        parsed.wrist_mode,
        parsed.distal_mode,
        parsed.segment_start,
        parsed.segment_end,
    )
    spin_thread = _spin_capture_node(node)
    try:
        input("Set pose 'thumb_natural_open' (手自然张开，大拇指标准自然张开), hold still, then press Enter...")
        print(f"Capturing thumb natural open for {parsed.duration:.1f}s...")
        open_vector = node.capture(parsed.duration)

        input("Set pose 'thumb_pinky_root_touch' (大拇指触碰小拇指指根), hold still, then press Enter...")
        print(f"Capturing thumb pinky root touch for {parsed.duration:.1f}s...")
        touch_vector = node.capture(parsed.duration)

        output = {
            "schema": "manus_l20.thumb_segment_frame.v1",
            "segment_start": int(parsed.segment_start),
            "segment_end": int(parsed.segment_end),
            "manus_open_vector": open_vector,
            "manus_touch_vector": touch_vector,
            "robot_open_command": robot_open_command,
            "robot_touch_command": robot_touch_command,
        }
        output_path = _save_yaml(output, parsed.output)
        print(yaml.safe_dump(output, sort_keys=False, allow_unicode=True))
        print(f"Saved thumb segment frame to {output_path}")
    finally:
        _shutdown_capture_node(node, spin_thread)


def _config_output_path(hand: str, filename: str, override: str) -> str:
    if str(override).strip():
        return str(Path(override).expanduser())
    root = _default_workspace_root()
    return str(root / "src" / "manus_l20_retarget" / "config" / filename.format(hand=hand))


def run_all(parsed: argparse.Namespace) -> None:
    hand = str(parsed.hand).strip().lower()
    transform = str(parsed.transform)
    if hand == "left" and transform == "right_glove_to_right_retarget":
        transform = "left_glove_to_right_retarget"
    flexion_output = _config_output_path(hand, "flexion_{hand}_calibration.yaml", parsed.flexion_output)
    yaw_output = _config_output_path(hand, "finger_yaw_{hand}_calibration.yaml", parsed.finger_yaw_output)
    thumb_flexion_output = _config_output_path(hand, "thumb_{hand}_flexion_mapping.yaml", parsed.thumb_flexion_output)
    thumb_frame_output = _config_output_path(hand, "thumb_segment_frame_{hand}.yaml", parsed.thumb_frame_output)

    rclpy.init()
    node = FullCalibrationCapture(
        parsed.glove_topic,
        transform,
        parsed.wrist_mode,
        parsed.distal_mode,
        parsed.segment_start,
        parsed.segment_end,
    )
    spin_thread = _spin_capture_node(node)
    labels = [
        ("natural_open", "自然张开；同时作为四指 open、yaw open、拇指 open、拇指 IK open"),
        ("four_finger_fist", "四指完全弯曲握拳；只用于四指弯曲 closed"),
        ("finger_close", "四指并拢；只用于四指 yaw close"),
        ("finger_spread", "四指外展；只用于四指 yaw spread"),
        ("thumb_pinky_root_touch", "大拇指触碰小拇指指根；同时用于拇指弯曲 touch、拇指 IK touch"),
    ]
    samples: dict[str, dict[str, Any]] = {}
    try:
        for label, prompt in labels:
            input(f"Set pose '{label}' ({prompt}), hold still, then press Enter...")
            print(f"Capturing '{label}' for {parsed.duration:.1f}s...")
            samples[label] = node.capture(parsed.duration)
            print(yaml.safe_dump({label: samples[label]}, sort_keys=False, allow_unicode=True))

        flexion_samples = {
            "open": samples["natural_open"]["flexion"],
            "four_finger_fist": samples["four_finger_fist"]["flexion"],
        }
        yaw_samples = {
            "natural_open": samples["natural_open"]["yaw"],
            "finger_close": samples["finger_close"]["yaw"],
            "finger_spread": samples["finger_spread"]["yaw"],
        }
        thumb_flexion_samples = {
            "thumb_pinky_root_touch": samples["thumb_pinky_root_touch"]["thumb_flexion"],
            "thumb_natural_open": samples["natural_open"]["thumb_flexion"],
        }

        flexion_calibration = _build_flexion_calibration(flexion_samples, flexion_output)
        yaw_calibration = _build_finger_yaw_calibration(yaw_samples, parsed.yaw_source, yaw_output)
        thumb_flexion_calibration = _build_thumb_flexion_calibration(thumb_flexion_samples, thumb_flexion_output)

        parsed_robot_open = _command_parameter(parsed.robot_open_command, [])
        robot_open_command = (
            _existing_top_level_command(thumb_frame_output, "robot_open_command", parsed_robot_open)
            if parsed_robot_open
            else _existing_top_level_command(thumb_frame_output, "robot_open_command", STANDARD_OPEN_COMMAND)
        )
        parsed_robot_touch = _command_parameter(parsed.robot_touch_command, [])
        if parsed_robot_touch:
            robot_touch_command = parsed_robot_touch
        else:
            touch_fallback = _command_parameter(
                thumb_flexion_calibration["command"].get("thumb_pinky_root_touch_command"),
                robot_open_command,
            )
            robot_touch_command = _existing_top_level_command(
                thumb_frame_output,
                "robot_touch_command",
                touch_fallback,
            )

        thumb_frame_calibration = {
            "schema": "manus_l20.thumb_segment_frame.v1",
            "segment_start": int(parsed.segment_start),
            "segment_end": int(parsed.segment_end),
            "manus_open_vector": samples["natural_open"]["thumb_segment_vector"],
            "manus_touch_vector": samples["thumb_pinky_root_touch"]["thumb_segment_vector"],
            "robot_open_command": robot_open_command,
            "robot_touch_command": robot_touch_command,
        }

        saved_paths = [
            _save_yaml(flexion_calibration, flexion_output),
            _save_yaml(yaw_calibration, yaw_output),
            _save_yaml(thumb_flexion_calibration, thumb_flexion_output),
            _save_yaml(thumb_frame_calibration, thumb_frame_output),
        ]
        print(
            yaml.safe_dump(
                {
                    "finger_yaw_selected_source": yaw_calibration["source"],
                    "finger_yaw_source_ranges": yaw_calibration["source_ranges"],
                    "saved": [str(path) for path in saved_paths],
                },
                sort_keys=False,
                allow_unicode=True,
            )
        )
    finally:
        _shutdown_capture_node(node, spin_thread)


def _add_common_glove_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--glove-topic", default="/manus_glove_0")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--transform", default="right_glove_to_right_retarget")
    parser.add_argument("--wrist-mode", default="estimate")
    parser.add_argument("--distal-mode", default="dip")


def _parse_key_list(value: str) -> list[str] | None:
    keys = [item.strip() for item in str(value).split(",") if item.strip()]
    if not keys:
        return None
    if len(keys) != 4:
        raise RuntimeError("--ergonomics-keys must contain exactly 4 comma-separated keys: index,middle,ring,pinky")
    return keys


def _build_parser() -> argparse.ArgumentParser:
    root = _default_workspace_root()
    config_root = root / "src" / "manus_l20_retarget" / "config"
    parser = argparse.ArgumentParser(description="Capture MANUS -> L20 calibration YAML files.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    flexion = subparsers.add_parser("flexion", description="Capture open/fist flexion calibration.")
    _add_common_glove_args(flexion)
    flexion.add_argument("--output", default=str(config_root / "flexion_right_calibration.yaml"))
    flexion.set_defaults(func=run_flexion)

    finger_yaw = subparsers.add_parser("finger-yaw", description="Capture four-finger yaw calibration.")
    _add_common_glove_args(finger_yaw)
    finger_yaw.add_argument(
        "--yaw-source",
        default="auto_orientation",
        choices=["auto", "auto_orientation", *ORIENTATION_AND_POSITION_SOURCES],
    )
    finger_yaw.add_argument("--output", default=str(config_root / "finger_yaw_right_calibration.yaml"))
    finger_yaw.set_defaults(func=run_finger_yaw)

    ergonomics_yaw = subparsers.add_parser(
        "ergonomics-yaw",
        description="Capture a non-invasive four-finger yaw calibration using MANUS ergonomics values.",
    )
    ergonomics_yaw.add_argument("--glove-topic", default="/manus_glove_0")
    ergonomics_yaw.add_argument("--duration", type=float, default=2.0)
    ergonomics_yaw.add_argument("--ergonomics-keys", default="")
    ergonomics_yaw.add_argument("--output", default=str(config_root / "finger_yaw_ergonomics_right_test.yaml"))
    ergonomics_yaw.set_defaults(func=run_finger_yaw_ergonomics)

    thumb_flexion = subparsers.add_parser("thumb-flexion", description="Capture thumb flexion mapping.")
    _add_common_glove_args(thumb_flexion)
    thumb_flexion.add_argument("--output", default=str(config_root / "thumb_right_flexion_mapping.yaml"))
    thumb_flexion.set_defaults(func=run_thumb_flexion)

    thumb_frame = subparsers.add_parser("thumb-frame", description="Capture two-pose thumb segment frame.")
    _add_common_glove_args(thumb_frame)
    thumb_frame.add_argument("--segment-start", type=int, default=2)
    thumb_frame.add_argument("--segment-end", type=int, default=3)
    thumb_frame.add_argument("--output", default=str(config_root / "thumb_segment_frame_right.yaml"))
    thumb_frame.add_argument("--thumb-flexion-mapping-path", default=str(config_root / "thumb_right_flexion_mapping.yaml"))
    thumb_frame.add_argument(
        "--robot-open-command",
        default="[254,248,246,249,254,128,115,111,131,188,221,255,255,255,255,254,254,254,254,254]",
    )
    thumb_frame.add_argument("--robot-touch-command", default="")
    thumb_frame.set_defaults(func=run_thumb_frame)

    all_calibration = subparsers.add_parser(
        "all",
        description="Capture shared poses once and write flexion/yaw/thumb calibration YAML files.",
    )
    _add_common_glove_args(all_calibration)
    all_calibration.add_argument("--hand", choices=["right", "left"], default="right")
    all_calibration.add_argument(
        "--yaw-source",
        default="auto_orientation",
        choices=["auto", "auto_orientation", *ORIENTATION_AND_POSITION_SOURCES],
    )
    all_calibration.add_argument("--segment-start", type=int, default=2)
    all_calibration.add_argument("--segment-end", type=int, default=3)
    all_calibration.add_argument("--flexion-output", default="")
    all_calibration.add_argument("--finger-yaw-output", default="")
    all_calibration.add_argument("--thumb-flexion-output", default="")
    all_calibration.add_argument("--thumb-frame-output", default="")
    all_calibration.add_argument("--robot-open-command", default="")
    all_calibration.add_argument("--robot-touch-command", default="")
    all_calibration.set_defaults(func=run_all)
    return parser


def main(args: list[str] | None = None) -> None:
    parser = _build_parser()
    parsed = parser.parse_args(args)
    parsed.func(parsed)


if __name__ == "__main__":
    main()
