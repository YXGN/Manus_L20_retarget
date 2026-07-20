from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


STANDARD_OPEN_COMMAND = [
    255,
    255,
    255,
    255,
    255,
    128,
    163,
    129,
    86,
    62,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
    255,
]

SLOT_LABELS = [
    "thumb_root",
    "index_root",
    "middle_root",
    "ring_root",
    "pinky_root",
    "thumb_roll",
    "index_yaw",
    "middle_yaw",
    "ring_yaw",
    "pinky_yaw",
    "thumb_yaw",
    "reserved_11",
    "reserved_12",
    "reserved_13",
    "reserved_14",
    "thumb_tip",
    "index_tip",
    "middle_tip",
    "ring_tip",
    "pinky_tip",
]


class G20JointProbe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("g20_joint_probe")
        self._pub = self.create_publisher(JointState, topic, 10)

    def publish_command(self, command: list[int]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = SLOT_LABELS
        msg.position = [float(value) for value in command]
        msg.velocity = [0.0] * 20
        msg.effort = [0.0] * 20
        self._pub.publish(msg)


def run_g20_probe(parsed: argparse.Namespace) -> None:
    rclpy.init()
    node = G20JointProbe(parsed.topic)
    try:
        base = list(STANDARD_OPEN_COMMAND)
        node.get_logger().info(f"publishing standard open command: {base}")
        node.publish_command(base)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(parsed.hold)

        for value in parsed.values:
            command = list(STANDARD_OPEN_COMMAND)
            command[parsed.slot] = max(0, min(255, int(value)))
            node.get_logger().info(
                f"probing slot {parsed.slot} ({SLOT_LABELS[parsed.slot]}) "
                f"with value {command[parsed.slot]}: {command}"
            )
            node.publish_command(command)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(parsed.hold)

        if parsed.return_open:
            node.get_logger().info("returning to standard open command")
            node.publish_command(base)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MANUS L20 debugging utilities.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    g20_probe = subparsers.add_parser("g20-probe", description="Probe one G20/L20 command slot on real hardware.")
    g20_probe.add_argument("--topic", default="/cb_right_hand_control_cmd")
    g20_probe.add_argument("--slot", type=int, required=True, choices=range(20))
    g20_probe.add_argument("--values", type=int, nargs="+", required=True)
    g20_probe.add_argument("--hold", type=float, default=1.5)
    g20_probe.add_argument("--return-open", action="store_true", default=True)
    g20_probe.set_defaults(func=run_g20_probe)
    return parser


def main(args: list[str] | None = None) -> None:
    parser = _build_parser()
    parsed = parser.parse_args(args)
    parsed.func(parsed)


if __name__ == "__main__":
    main()
