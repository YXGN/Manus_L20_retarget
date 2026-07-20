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
    _joint_flexion_rad,
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
POSITION_SOURCES = ("pip", "dip", "tip")
ORIENTATION_SOURCES = ("mcp_orientation", "pip_orientation", "ip_orientation", "dip_orientation")
ORIENTATION_AND_POSITION_SOURCES = (*POSITION_SOURCES, *ORIENTATION_SOURCES)


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


def _flexion_angles(landmarks: Any) -> dict[str, list[float]]:
    root_angles: list[float] = []
    tip_angles: list[float] = []
    for mcp, pip, dip, tip in FINGER_LANDMARKS:
        root_angles.append(_joint_flexion_rad(landmarks[mcp], landmarks[pip], landmarks[dip]))
        tip_angles.append(_joint_flexion_rad(landmarks[pip], landmarks[dip], landmarks[tip]))
    return {
        "root_rad": root_angles,
        "tip_rad": tip_angles,
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


def _build_flexion_calibration(samples: dict[str, dict[str, list[float]]], output_path: str) -> dict[str, Any]:
    open_sample = samples["open"]
    four_finger_sample = samples["four_finger_fist"]
    root_closed = [open_sample["root_rad"][0], *four_finger_sample["root_rad"][1:]]
    tip_closed = [open_sample["tip_rad"][0], *four_finger_sample["tip_rad"][1:]]
    command = _existing_command(output_path) or {
        "open_command": DEFAULT_L20_OPEN_COMMAND,
        "four_finger_closed_command": DEFAULT_L20_FOUR_FINGER_CLOSED_COMMAND,
    }
    return {
        "schema": "manus_l20.flexion_calibration.v1",
        "finger_order": ["thumb", "index", "middle", "ring", "pinky"],
        "root_flexion_open_rad": open_sample["root_rad"],
        "root_flexion_closed_rad": root_closed,
        "tip_flexion_open_rad": open_sample["tip_rad"],
        "tip_flexion_closed_rad": tip_closed,
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


def _build_thumb_flexion_calibration(samples: dict[str, dict[str, float]], output_path: str) -> dict[str, Any]:
    command = _existing_command(output_path) or {
        "thumb_natural_open_command": DEFAULT_THUMB_NATURAL_OPEN_COMMAND,
        "thumb_pinky_root_touch_command": None,
    }
    return {
        "schema": "manus_l20.thumb_flexion_mapping.v1",
        "finger": "thumb",
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
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)
    return output_path


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


def _add_common_glove_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--glove-topic", default="/manus_glove_0")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--wrist-mode", default="estimate")
    parser.add_argument("--distal-mode", default="dip")


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

    thumb_flexion = subparsers.add_parser("thumb-flexion", description="Capture thumb flexion mapping.")
    _add_common_glove_args(thumb_flexion)
    thumb_flexion.add_argument("--output", default=str(config_root / "thumb_right_flexion_mapping.yaml"))
    thumb_flexion.set_defaults(func=run_thumb_flexion)

    thumb_open = subparsers.add_parser("thumb-open-vector", description="Capture thumb segment open vector.")
    _add_common_glove_args(thumb_open)
    thumb_open.add_argument("--segment-start", type=int, default=2)
    thumb_open.add_argument("--segment-end", type=int, default=3)
    thumb_open.add_argument("--output", default=str(config_root / "thumb_segment_open_vector_right.yaml"))
    thumb_open.set_defaults(func=run_thumb_open_vector)

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
    return parser


def main(args: list[str] | None = None) -> None:
    parser = _build_parser()
    parsed = parser.parse_args(args)
    parsed.func(parsed)


if __name__ == "__main__":
    main()
