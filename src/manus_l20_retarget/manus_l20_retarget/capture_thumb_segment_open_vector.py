from __future__ import annotations

import argparse
import statistics
import threading
import time
from pathlib import Path

import rclpy
import yaml
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node

from .manus_landmarks import manus_raw_nodes_to_mediapipe_landmarks
from .manus_l20_retarget_node import _default_workspace_root, _manus_thumb_local_point_at


class ThumbSegmentOpenVectorCapture(Node):
    def __init__(
        self,
        glove_topic: str,
        transform: str,
        wrist_mode: str,
        distal_mode: str,
        segment_start: int,
        segment_end: int,
    ) -> None:
        super().__init__("manus_l20_thumb_segment_open_vector_capture")
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


def _mean_vector(samples: list[list[float]]) -> list[float]:
    length = min(len(sample) for sample in samples)
    return [
        round(float(statistics.fmean(sample[index] for sample in samples)), 6)
        for index in range(length)
    ]


def _parse_args() -> argparse.Namespace:
    root = _default_workspace_root()
    default_output = root / "src" / "manus_l20_retarget" / "config" / "thumb_segment_open_vector_right.yaml"
    parser = argparse.ArgumentParser(description="Capture fixed MANUS thumb segment open vector for L20/G20 segment IK.")
    parser.add_argument("--glove-topic", default="/manus_glove_0")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--wrist-mode", default="estimate")
    parser.add_argument("--distal-mode", default="dip")
    parser.add_argument("--segment-start", type=int, default=2)
    parser.add_argument("--segment-end", type=int, default=3)
    parser.add_argument("--output", default=str(default_output))
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = ThumbSegmentOpenVectorCapture(
        parsed.glove_topic,
        parsed.transform,
        parsed.wrist_mode,
        parsed.distal_mode,
        parsed.segment_start,
        parsed.segment_end,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
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
        output_path = Path(parsed.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(output, handle, sort_keys=False, allow_unicode=True)
        print(yaml.safe_dump(output, sort_keys=False, allow_unicode=True))
        print(f"Saved thumb segment open vector to {output_path}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
