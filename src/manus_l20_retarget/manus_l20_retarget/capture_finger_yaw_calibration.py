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


DEFAULT_L20_NATURAL_OPEN_COMMAND = [
    255, 255, 255, 255, 255,
    128, 163, 129, 86, 62, 255,
    255, 255, 255, 255,
    255, 255, 255, 255, 255,
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
POSITION_SOURCES = ("pip", "dip", "tip")
ORIENTATION_SOURCES = ("mcp_orientation", "pip_orientation", "ip_orientation", "dip_orientation")
ORIENTATION_AND_POSITION_SOURCES = (*POSITION_SOURCES, *ORIENTATION_SOURCES)


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
            for source in ("pip", "dip", "tip")
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
        if self._yaw_source in all_yaw_rad:
            selected_source = self._yaw_source
        else:
            selected_source = "pip"
        return {
            "yaw_rad": all_yaw_rad[selected_source],
            "all_yaw_rad": all_yaw_rad,
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


def _build_calibration(samples: dict[str, dict[str, Any]], yaw_source: str, output_path: str) -> dict[str, Any]:
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
        "natural_open_command": DEFAULT_L20_NATURAL_OPEN_COMMAND,
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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture MANUS four-finger yaw calibration for L20 mapping.")
    parser.add_argument("--glove-topic", default="/manus_glove_0")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--wrist-mode", default="estimate")
    parser.add_argument("--distal-mode", default="dip")
    parser.add_argument(
        "--yaw-source",
        default="auto_orientation",
        choices=["auto", "auto_orientation", *ORIENTATION_AND_POSITION_SOURCES],
    )
    parser.add_argument(
        "--output",
        default="/home/huangzizhe/Manus_L20_retarget/src/manus_l20_retarget/config/finger_yaw_right_calibration.yaml",
    )
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = FingerYawCalibrationCapture(
        parsed.glove_topic,
        parsed.transform,
        parsed.wrist_mode,
        parsed.distal_mode,
        parsed.yaw_source,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
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
        calibration = _build_calibration(samples, parsed.yaw_source, parsed.output)
        output_path = Path(parsed.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(calibration, handle, sort_keys=False, allow_unicode=True)
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
