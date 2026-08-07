#!/usr/bin/env python3
"""Pipe Qt commands to the MANUS publisher without sharing an rclpy context."""

from __future__ import annotations

import select
import sys

import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node
from std_msgs.msg import String

SKELETON_HZ = 30.0


class CalibrationBridge(Node):
    def __init__(self) -> None:
        super().__init__("manus_calibration_qt_bridge")
        self.publisher = self.create_publisher(String, "/manus/calibration/command", 10)
        self.subscription = self.create_subscription(
            String,
            "/manus/calibration/status",
            lambda message: print(message.data, flush=True),
            20,
        )
        self.skeleton_subscriptions = [
            self.create_subscription(
                ManusGlove,
                topic,
                lambda message, value=hand: self._cache_skeleton(value, message),
                1,
            )
            for hand, topic in (("right", "/manus_glove_0"), ("left", "/manus_glove_1"))
        ]
        self.skeletons: dict[str, ManusGlove] = {}
        self.skeleton_dirty: set[str] = set()
        self.skeleton_timer = self.create_timer(1.0 / SKELETON_HZ, self._publish_skeletons)

    def send(self, command: str) -> None:
        message = String()
        message.data = command
        self.publisher.publish(message)

    def _cache_skeleton(self, hand: str, message: ManusGlove) -> None:
        self.skeletons[hand] = message
        self.skeleton_dirty.add(hand)

    def _publish_skeletons(self) -> None:
        for hand in tuple(self.skeleton_dirty):
            message = self.skeletons.get(hand)
            if message is None:
                continue
            payload = ";".join(
                f"{node.node_id},{node.parent_node_id},"
                f"{node.pose.position.x:.6f},{node.pose.position.y:.6f},{node.pose.position.z:.6f}"
                for node in message.raw_nodes
            )
            print(f"SKELETON\t{hand}\t{payload}", flush=True)
        self.skeleton_dirty.clear()


def main() -> int:
    rclpy.init()
    node = CalibrationBridge()
    print("BRIDGE_READY", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if not readable:
                continue
            line = sys.stdin.readline()
            if not line:
                break
            node.send(line.rstrip("\n"))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
