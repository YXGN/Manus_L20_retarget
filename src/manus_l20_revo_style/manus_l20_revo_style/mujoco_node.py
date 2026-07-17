from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .core import load_yaml
from .mujoco_sim import L20MujocoSim, PassiveViewer
from .paths import resolve_workspace_path


class L20MujocoNode(Node):
    def __init__(self) -> None:
        super().__init__("manus_l20_revo_style_mujoco")
        default_config = str(resolve_workspace_path("src/manus_l20_revo_style/config/revo_style_l20_right.yaml"))
        default_mjcf = str(resolve_workspace_path("src/l20_thumb_ik/assets/mjcf/linkerhand_l20_right/model.xml"))
        self.declare_parameter("config_path", default_config)
        self.declare_parameter("mjcf_path", default_mjcf)
        self.declare_parameter("command_topic", "/manus_l20_revo_style_right/l20_command")
        self.declare_parameter("joint_state_topic", "~/joint_states")
        self.declare_parameter("target_joint_state_topic", "~/target_joint_states")
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("sim_substeps", 4)
        self.declare_parameter("enable_viewer", False)

        config_path = Path(str(self.get_parameter("config_path").value).strip() or default_config).expanduser().resolve()
        mjcf_path = Path(str(self.get_parameter("mjcf_path").value).strip() or default_mjcf).expanduser().resolve()
        self._sim = L20MujocoSim(str(mjcf_path), load_yaml(config_path))
        self._viewer = PassiveViewer(self._sim) if bool(self.get_parameter("enable_viewer").value) else None
        self._latest_command: list[int] | None = None
        self.create_subscription(JointState, str(self.get_parameter("command_topic").value), self._on_command, 10)
        self._joint_pub = self.create_publisher(JointState, str(self.get_parameter("joint_state_topic").value), 10)
        self._target_pub = self.create_publisher(JointState, str(self.get_parameter("target_joint_state_topic").value), 10)
        period_s = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(period_s, self._on_timer)
        self.get_logger().info(
            "L20 MuJoCo simulation ready: "
            f"command={self.get_parameter('command_topic').value}, mjcf={mjcf_path}"
        )

    def destroy_node(self) -> bool:
        if self._viewer is not None:
            self._viewer.close()
        return super().destroy_node()

    def _on_command(self, msg: JointState) -> None:
        if len(msg.position) < 20:
            self.get_logger().warning(f"ignoring short L20 command with {len(msg.position)} values")
            return
        command = [int(round(float(value))) for value in msg.position[:20]]
        self._latest_command = command
        self._sim.set_l20_command(command)
        self._publish_target()

    def _on_timer(self) -> None:
        self._sim.step(int(self.get_parameter("sim_substeps").value))
        self._publish_measured()
        if self._viewer is not None:
            if self._viewer.is_running:
                self._viewer.sync()
            else:
                self.get_logger().warning("MuJoCo viewer closed")
                self._viewer = None

    def _publish_measured(self) -> None:
        self._publish_joint_dict(self._joint_pub, self._sim.measured_joint_positions())

    def _publish_target(self) -> None:
        self._publish_joint_dict(self._target_pub, self._sim.target_joint_positions())

    def _publish_joint_dict(self, pub, values: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(values.keys())
        msg.position = list(values.values())
        pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = L20MujocoNode()
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
