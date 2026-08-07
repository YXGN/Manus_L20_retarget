from __future__ import annotations

import math

import rclpy
from manus_ros2_msgs.msg import ManusErgonomics, ManusGlove
from rclpy.node import Node


class MockManusPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_manus_publisher")
        self.declare_parameter("topic", "/manus_glove_0")
        self.declare_parameter("side", "right")
        self.declare_parameter("rate_hz", 30.0)
        self._publisher = self.create_publisher(ManusGlove, self.get_parameter("topic").value, 10)
        self._count = 0
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 1.0), self._publish)
        self.get_logger().info("mock MANUS publisher started")

    def _publish(self) -> None:
        phase = self._count / 30.0
        flex = 0.5 + 0.5 * math.sin(phase * math.tau * 0.25)
        spread = 0.5 * math.sin(phase * math.tau * 0.13)
        prefix = "RightFinger" if self.get_parameter("side").value == "right" else "LeftFinger"
        msg = ManusGlove()
        msg.glove_id = 0
        msg.side = str(self.get_parameter("side").value)
        ergonomics = []
        for label in ("Thumb", "Index", "Middle", "Ring", "Pinky"):
            ergonomics.append(_ergo(f"{prefix}{label}MCPStretch", flex))
            ergonomics.append(_ergo(f"{prefix}{label}PIPStretch", flex))
            ergonomics.append(_ergo(f"{prefix}{label}DIPStretch", flex * 0.85))
            ergonomics.append(_ergo(f"{prefix}{label}MCPSpread", spread))
        msg.ergonomics = ergonomics
        msg.ergonomics_count = len(ergonomics)
        self._publisher.publish(msg)
        self._count += 1


def _ergo(name: str, value: float) -> ManusErgonomics:
    msg = ManusErgonomics()
    msg.type = name
    msg.value = float(value)
    return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockManusPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
