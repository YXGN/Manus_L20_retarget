from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


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


def _command_from_yaml(path: str, key: str) -> list[int]:
    yaml_path = Path(path).expanduser().resolve()
    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    value = _lookup_yaml_key(data, key)
    if not isinstance(value, list):
        raise ValueError(f"{yaml_path}:{key} is not a list")
    if len(value) != 20:
        raise ValueError(f"{yaml_path}:{key} must contain 20 values, got {len(value)}")
    return [max(0, min(255, int(round(float(item))))) for item in value]


def _lookup_yaml_key(data: object, key: str) -> object:
    if not key:
        raise ValueError("--key is required when using --yaml")
    if isinstance(data, dict):
        if key in data:
            return data[key]
        command_block = data.get("command")
        if isinstance(command_block, dict) and key in command_block:
            return command_block[key]
    current = data
    for part in key.split("."):
        if not isinstance(current, dict) or part not in current:
            raise KeyError(f"YAML key not found: {key}")
        current = current[part]
    return current


def run_send_command(parsed: argparse.Namespace) -> None:
    if parsed.yaml:
        command = _command_from_yaml(parsed.yaml, parsed.key)
    else:
        command = [max(0, min(255, int(value))) for value in parsed.values]
        if len(command) != 20:
            raise ValueError(f"--values must contain 20 values, got {len(command)}")

    rclpy.init()
    node = G20JointProbe(parsed.topic)
    try:
        node.get_logger().info(f"publishing L20 command to {parsed.topic}: {command}")
        for _ in range(max(1, parsed.repeat)):
            node.publish_command(command)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(max(0.0, parsed.period))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


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

    send_command = subparsers.add_parser(
        "send-command",
        description="Publish one full 20-slot L20/G20 command to the hardware command topic.",
    )
    send_command.add_argument("--topic", default="/cb_right_hand_control_cmd")
    source = send_command.add_mutually_exclusive_group(required=True)
    source.add_argument("--values", type=int, nargs=20)
    source.add_argument("--yaml", help="Calibration YAML file containing a 20-value command list.")
    send_command.add_argument("--key", default="", help="Command key, e.g. open_command or command.open_command.")
    send_command.add_argument("--repeat", type=int, default=5)
    send_command.add_argument("--period", type=float, default=0.05)
    send_command.set_defaults(func=run_send_command)
    return parser


def main(args: list[str] | None = None) -> None:
    parser = _build_parser()
    parsed = parser.parse_args(args)
    parsed.func(parsed)


if __name__ == "__main__":
    main()
