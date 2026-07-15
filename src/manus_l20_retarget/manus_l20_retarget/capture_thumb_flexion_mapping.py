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


DEFAULT_THUMB_NATURAL_OPEN_COMMAND = [
    99, 255, 255, 255, 255,
    0, 163, 129, 86, 62, 222,
    255, 255, 255, 255,
    100, 255, 255, 255, 255,
]


class ThumbFlexionMappingCapture(Node):
    def __init__(
        self,
        glove_topic: str,
        transform: str,
        wrist_mode: str,
        distal_mode: str,
    ) -> None:
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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture two-pose MANUS thumb flexion mapping calibration.")
    parser.add_argument("--glove-topic", default="/manus_glove_0")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--wrist-mode", default="estimate")
    parser.add_argument("--distal-mode", default="dip")
    parser.add_argument(
        "--output",
        default="/home/huangzizhe/Manus_L20_retarget/src/manus_l20_retarget/config/thumb_right_flexion_mapping.yaml",
    )
    return parser.parse_args()


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


def _build_calibration(samples: dict[str, dict[str, float]], output_path: str) -> dict[str, Any]:
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


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = ThumbFlexionMappingCapture(
        parsed.glove_topic,
        parsed.transform,
        parsed.wrist_mode,
        parsed.distal_mode,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
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
        calibration = _build_calibration(samples, parsed.output)
        output_path = Path(parsed.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(calibration, handle, sort_keys=False, allow_unicode=True)
        print(f"Saved thumb flexion mapping calibration to {output_path}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
