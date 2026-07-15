from __future__ import annotations

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Point
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node
from visualization_msgs.msg import Marker

from .manus_landmarks import manus_raw_nodes_to_mediapipe_landmarks, summarize_landmarks


class ManusLandmarkInspector(Node):
    def __init__(
        self,
        topic: str,
        interval_sec: float,
        transform: str,
        wrist_mode: str,
        distal_mode: str,
    ) -> None:
        super().__init__("manus_landmark_inspector")
        self._interval_sec = max(0.1, float(interval_sec))
        self._transform = transform
        self._wrist_mode = wrist_mode
        self._distal_mode = distal_mode
        self._last_print_time = 0.0
        self._marker_pub = self.create_publisher(Marker, "/manus/right_landmarks", 10)
        self.create_subscription(ManusGlove, topic, self._on_glove, 10)

    def _on_glove(self, msg: ManusGlove) -> None:
        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=self._transform,
            wrist_mode=self._wrist_mode,
            distal_mode="dip" if self._distal_mode == "both" else self._distal_mode,
        )
        self._publish_marker(landmarks)
        now = time.monotonic()
        if now - self._last_print_time < self._interval_sec:
            return
        self._last_print_time = now
        if self._distal_mode == "both":
            modes = {}
            for distal_mode in ("dip", "ip"):
                mode_landmarks = manus_raw_nodes_to_mediapipe_landmarks(
                    msg.raw_nodes,
                    transform=self._transform,
                    wrist_mode=self._wrist_mode,
                    distal_mode=distal_mode,
                )
                modes[distal_mode] = summarize_landmarks(mode_landmarks)
            result = {
                "side": msg.side,
                "glove_id": int(msg.glove_id),
                "transform": self._transform,
                "wrist_mode": self._wrist_mode,
                "distal_mode": "both",
                "modes": modes,
            }
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return

        result = {
            "side": msg.side,
            "glove_id": int(msg.glove_id),
            "transform": self._transform,
            "wrist_mode": self._wrist_mode,
            "distal_mode": self._distal_mode,
            **summarize_landmarks(landmarks),
        }
        print(json.dumps(result, ensure_ascii=False, indent=2))

    def _publish_marker(self, landmarks) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "manus_glove"
        marker.ns = "manus_landmarks"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.points = [Point(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in landmarks]
        self._marker_pub.publish(marker)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect MANUS raw_nodes converted to MediaPipe-style landmarks.")
    parser.add_argument("--topic", default="/manus_glove_0")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--wrist-mode", default="estimate", choices=("estimate", "palm_center"))
    parser.add_argument("--distal-mode", default="dip", choices=("dip", "ip", "both"))
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = ManusLandmarkInspector(
        parsed.topic,
        parsed.interval,
        parsed.transform,
        parsed.wrist_mode,
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
