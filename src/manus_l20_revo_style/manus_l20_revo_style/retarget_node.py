from __future__ import annotations

from pathlib import Path

import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .core import RevoStyleL20Retarget, load_yaml
from .paths import resolve_workspace_path


class RevoStyleL20Node(Node):
    def __init__(self) -> None:
        super().__init__("manus_l20_revo_style")
        default_config = str(resolve_workspace_path("src/manus_l20_revo_style/config/revo_style_l20_right.yaml"))
        self.declare_parameter("config_path", default_config)
        self.declare_parameter("input_topic", "/manus_glove_0")
        self.declare_parameter("command_topic", "~/l20_command")
        self.declare_parameter("target_topic", "~/retarget_targets")
        self.declare_parameter("publish_rate_hz", 120.0)
        self.declare_parameter("require_matching_side", True)
        self.declare_parameter("log_every_n", 30)

        config_path = Path(str(self.get_parameter("config_path").value).strip() or default_config).expanduser().resolve()
        self._config = load_yaml(config_path)
        self._retarget = RevoStyleL20Retarget(self._config)
        self._side = str(self._config.get("input", {}).get("side", "right")).lower()
        self._latest_command: list[int] | None = None
        self._latest_targets: dict[str, float] | None = None
        self._sequence = 0
        self._publish_count = 0

        self._command_pub = self.create_publisher(JointState, str(self.get_parameter("command_topic").value), 10)
        self._target_pub = self.create_publisher(JointState, str(self.get_parameter("target_topic").value), 10)
        self.create_subscription(ManusGlove, str(self.get_parameter("input_topic").value), self._on_glove, 10)
        period_s = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(period_s, self._publish_latest)
        self.get_logger().info(
            "Revo-style L20 retarget ready: "
            f"input={self.get_parameter('input_topic').value}, "
            f"command={self.get_parameter('command_topic').value}, config={config_path}"
        )

    def _on_glove(self, msg: ManusGlove) -> None:
        if bool(self.get_parameter("require_matching_side").value):
            msg_side = str(msg.side).strip().lower()
            if msg_side and msg_side not in {self._side, self._side[:1]}:
                return
        ergonomics = {entry.type: float(entry.value) for entry in msg.ergonomics if entry.type}
        if not ergonomics:
            return
        targets, command = self._retarget.retarget(ergonomics, smooth=True)
        self._latest_targets = targets
        self._latest_command = command
        self._sequence += 1
        log_every_n = max(0, int(self.get_parameter("log_every_n").value))
        if log_every_n and self._sequence % log_every_n == 0:
            preview = {key: round(value, 3) for key, value in list(targets.items())[:6]}
            self.get_logger().info(f"seq={self._sequence} command={command} targets={preview}")

    def _publish_latest(self) -> None:
        if self._latest_command is None or self._latest_targets is None:
            return
        stamp = self.get_clock().now().to_msg()

        command_msg = JointState()
        command_msg.header.stamp = stamp
        command_msg.name = [f"l20_slot_{index:02d}" for index in range(len(self._latest_command))]
        command_msg.position = [float(value) for value in self._latest_command]
        self._command_pub.publish(command_msg)

        target_msg = JointState()
        target_msg.header.stamp = stamp
        target_msg.name = list(self._latest_targets.keys())
        target_msg.position = list(self._latest_targets.values())
        self._target_pub.publish(target_msg)
        self._publish_count += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RevoStyleL20Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
