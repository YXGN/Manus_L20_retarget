from __future__ import annotations

import argparse
import json
import time

import numpy as np
import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node

from .manus_landmarks import (
    FINGER_CHAINS,
    TRANSFORMS,
    _ordered_chain,
    _select_four_landmarks,
    ordered_chain_debug,
)


class ManusFingerPointInspector(Node):
    def __init__(
        self,
        topic: str,
        interval_sec: float,
        transform: str,
        distal_mode: str,
    ) -> None:
        super().__init__("manus_finger_point_inspector")
        self._interval_sec = max(0.1, float(interval_sec))
        self._transform = transform
        self._distal_mode = distal_mode
        self._last_print_time = 0.0
        self.create_subscription(ManusGlove, topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        now = time.monotonic()
        if now - self._last_print_time < self._interval_sec:
            return
        self._last_print_time = now

        matrix = TRANSFORMS.get(self._transform)
        if matrix is None:
            raise ValueError(f"unknown MANUS landmark transform: {self._transform}")

        output = {
            "side": msg.side,
            "glove_id": int(msg.glove_id),
            "transform": self._transform,
            "distal_mode": self._distal_mode,
            "fingers": {},
        }
        for chain in FINGER_CHAINS:
            raw_points = _ordered_chain(msg.raw_nodes, chain)
            if len(raw_points) < 2:
                continue
            selected_points = _select_four_landmarks(
                raw_points,
                chain=chain,
                distal_mode=self._distal_mode,
            )
            output["fingers"][chain] = {
                "raw_count": len(raw_points),
                "raw_order": ordered_chain_debug(msg.raw_nodes, chain),
                "raw_positions": _round_points(raw_points),
                "selected_four_positions": _round_points(selected_points),
                "selected_four_transformed": _round_points([point @ matrix.T for point in selected_points]),
                "selected_four_labels": _selected_labels(chain, self._distal_mode, len(raw_points)),
            }

        print(json.dumps(output, ensure_ascii=False, indent=2))


def _round_points(points: list[np.ndarray]) -> list[list[float]]:
    return [np.round(np.asarray(point, dtype=np.float64), 6).tolist() for point in points]


def _selected_labels(chain: str, distal_mode: str, count: int) -> list[str]:
    if count >= 5:
        if chain != "Thumb" and distal_mode == "ip":
            return ["MCP", "PIP", "IP", "TIP"]
        return ["MCP", "PIP", "DIP", "TIP"]
    if count == 4:
        return ["point0", "point1", "point2", "point3"]
    return ["point0", "point1", "last", "last"]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print MANUS raw finger points and selected 4-point landmarks.")
    parser.add_argument("--topic", default="/manus_glove_0")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--distal-mode", default="dip", choices=("dip", "ip"))
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = ManusFingerPointInspector(
        parsed.topic,
        parsed.interval,
        parsed.transform,
        parsed.distal_mode,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
