from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from manus_ros2_msgs.msg import ManusVibrationCommand

from .haptic_mapper import HapticMapper, HapticMappingConfig


class HapticFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("manus_l20_haptic_feedback")

        self.declare_parameter("glove_id", 0)
        self.declare_parameter("force_topic", "/manus_l20_haptics/force")
        self.declare_parameter("vib_topic", "")
        self.declare_parameter("normal_force_full_scale", 100.0)
        self.declare_parameter("approach_full_scale", 200.0)
        self.declare_parameter("attack_alpha", 0.35)
        self.declare_parameter("release_alpha", 0.10)
        self.declare_parameter("contact_threshold", 0.05)
        self.declare_parameter("use_approach", False)
        self.declare_parameter("max_intensity", 1.0)
        self.declare_parameter("invert_fingers", False)

        glove_id = int(self.get_parameter("glove_id").value)
        force_topic = str(self.get_parameter("force_topic").value)
        vib_topic = str(self.get_parameter("vib_topic").value)
        if not vib_topic:
            vib_topic = f"/manus_glove_{glove_id}/vibration_cmd"

        self._mapper = HapticMapper(
            HapticMappingConfig(
                normal_force_full_scale=float(self.get_parameter("normal_force_full_scale").value),
                approach_full_scale=float(self.get_parameter("approach_full_scale").value),
                attack_alpha=float(self.get_parameter("attack_alpha").value),
                release_alpha=float(self.get_parameter("release_alpha").value),
                contact_threshold=float(self.get_parameter("contact_threshold").value),
                use_approach=bool(self.get_parameter("use_approach").value),
                max_intensity=float(self.get_parameter("max_intensity").value),
                invert_fingers=bool(self.get_parameter("invert_fingers").value),
            )
        )

        self._pub = self.create_publisher(ManusVibrationCommand, vib_topic, 10)
        self._sub = self.create_subscription(Float32MultiArray, force_topic, self._on_force, 10)

        self.get_logger().info(
            f"L20 haptics feedback: {force_topic} -> {vib_topic} "
            f"(glove_id={glove_id}, full_scale={self._mapper.config.normal_force_full_scale})"
        )

    def _on_force(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        normal_force = data[0:5]
        approach_inc = data[15:20]
        intensities = self._mapper.update(normal_force, approach_inc)
        cmd = ManusVibrationCommand()
        cmd.intensities = intensities
        self._pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HapticFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
