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

from .manus_landmarks import manus_raw_nodes_to_mediapipe_landmarks
from .manus_l20_retarget_node import FINGER_LANDMARKS, _joint_flexion_rad


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
class FlexionCalibrationCapture(Node):
    def __init__(
        self,
        glove_topic: str,
        transform: str,
        wrist_mode: str,
        distal_mode: str,
    ) -> None:
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


def _build_calibration(samples: dict[str, dict[str, list[float]]], output_path: str) -> dict[str, Any]:
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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture MANUS open/closed flexion calibration for L20 mapping.")
    parser.add_argument("--glove-topic", default="/manus_glove_0")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--wrist-mode", default="estimate")
    parser.add_argument("--distal-mode", default="dip")
    parser.add_argument(
        "--output",
        default="/home/huangzizhe/Manus_L20_retarget/src/manus_l20_retarget/config/flexion_right_calibration.yaml",
    )
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = FlexionCalibrationCapture(
        parsed.glove_topic,
        parsed.transform,
        parsed.wrist_mode,
        parsed.distal_mode,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
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
        calibration = _build_calibration(samples, parsed.output)
        output_path = Path(parsed.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(calibration, handle, sort_keys=False, allow_unicode=True)
        print(f"Saved flexion calibration to {output_path}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
