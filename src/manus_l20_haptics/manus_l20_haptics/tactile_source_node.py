from __future__ import annotations

import math
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from linker_hand_ros2_sdk.LinkerHand.linker_hand_api import LinkerHandApi
except Exception as exc:  # pragma: no cover - exercised only when SDK is missing
    LinkerHandApi = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class TactileSourceNode(Node):
    def __init__(self) -> None:
        super().__init__("linkerhand_l20_tactile_source")

        self.declare_parameter("enabled", True)
        self.declare_parameter("mock", False)
        self.declare_parameter("hand_joint", "L20")
        self.declare_parameter("hand_type", "left")
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("poll_rate_hz", 30.0)
        self.declare_parameter("force_topic", "/manus_l20_haptics/force")
        self.declare_parameter("warn_poll_ms", 50.0)
        self.declare_parameter("read_mode", "auto")
        self.declare_parameter("matrix_reduce", "max")
        self.declare_parameter("matrix_sleep_sec", 0.009)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._mock = bool(self.get_parameter("mock").value)
        self._rate = max(1.0, float(self.get_parameter("poll_rate_hz").value))
        self._warn_poll_ms = max(0.0, float(self.get_parameter("warn_poll_ms").value))
        self._read_mode = str(self.get_parameter("read_mode").value).lower()
        self._matrix_reduce = str(self.get_parameter("matrix_reduce").value).lower()
        self._matrix_sleep_sec = max(0.0, float(self.get_parameter("matrix_sleep_sec").value))
        force_topic = str(self.get_parameter("force_topic").value)

        self._pub = self.create_publisher(Float32MultiArray, force_topic, 10)
        self._hand: Any | None = None
        self._init_thread: threading.Thread | None = None
        self._timer = None

        if not self._enabled:
            self.get_logger().warn("L20 tactile source disabled by parameter")
            return

        if self._mock:
            self.get_logger().info(f"L20 tactile source mock mode publishing {force_topic}")
            self._timer = self.create_timer(1.0 / self._rate, self._mock_tick)
            return

        if LinkerHandApi is None:
            self.get_logger().error(
                f"linker_hand_ros2_sdk import failed: {_IMPORT_ERROR!r}. "
                "Set mock:=true to test the MANUS haptic path without CAN hardware."
            )
            return

        self._init_thread = threading.Thread(target=self._init_hand, daemon=True)
        self._init_thread.start()

    def _init_hand(self) -> None:
        hand_joint = str(self.get_parameter("hand_joint").value)
        hand_type = str(self.get_parameter("hand_type").value)
        can_channel = str(self.get_parameter("can_channel").value)
        try:
            self.get_logger().info(
                f"Initializing LinkerHand tactile reader: joint={hand_joint} "
                f"type={hand_type} can={can_channel}"
            )
            self._hand = LinkerHandApi(
                hand_joint=hand_joint,
                hand_type=hand_type,
                can=can_channel,
            )
            touch_type = self._detect_touch_type()
            if self._read_mode == "auto":
                self._read_mode = "matrix" if touch_type > 1 else "force"
            self.get_logger().info(
                f"LinkerHand tactile reader ready at {self._rate:.1f} Hz "
                f"(touch_type={touch_type}, read_mode={self._read_mode})"
            )
            self._timer = self.create_timer(1.0 / self._rate, self._tick)
        except Exception as exc:
            self.get_logger().error(f"LinkerHand tactile init failed: {exc!r}")

    def _tick(self) -> None:
        if self._hand is None:
            return
        started = time.monotonic()
        try:
            data = self._read_tactile()
            msg = self._force_msg(data)
            self._pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"LinkerHand get_force failed: {exc!r}")
            return

        elapsed_ms = (time.monotonic() - started) * 1000.0
        if self._warn_poll_ms and elapsed_ms > self._warn_poll_ms:
            self.get_logger().warn(f"LinkerHand get_force took {elapsed_ms:.1f} ms")

    def _mock_tick(self) -> None:
        t = time.monotonic()
        wave = 0.5 + 0.5 * math.sin(2.0 * math.pi * 0.35 * t)
        normal = [wave * scale * 100.0 for scale in (1.0, 0.9, 0.95, 0.85, 0.75)]
        msg = self._force_msg([normal, [v * 0.2 for v in normal], [0.0] * 5, normal])
        self._pub.publish(msg)

    def _detect_touch_type(self) -> int:
        try:
            return int(self._hand.get_touch_type())
        except Exception as exc:
            self.get_logger().warn(f"Could not read LinkerHand touch type: {exc!r}")
            return -1

    def _read_tactile(self) -> list[list[float]]:
        if self._read_mode == "matrix":
            normal = self._read_matrix_force()
            return [normal, [0.0] * 5, [0.0] * 5, normal]
        if self._read_mode == "touch":
            touch = self._array5(self._hand.get_touch())
            normal = [100.0 if value > 0.0 else 0.0 for value in touch]
            return [normal, [0.0] * 5, [0.0] * 5, normal]
        return self._hand.get_force()

    def _read_matrix_force(self) -> list[float]:
        readers = (
            self._hand.get_thumb_matrix_touch,
            self._hand.get_index_matrix_touch,
            self._hand.get_middle_matrix_touch,
            self._hand.get_ring_matrix_touch,
            self._hand.get_little_matrix_touch,
        )
        return [self._reduce_matrix(reader(sleep_time=self._matrix_sleep_sec)) for reader in readers]

    def _reduce_matrix(self, matrix: Any) -> float:
        values: list[float] = []
        for row in matrix:
            try:
                values.extend(float(value) for value in row)
            except TypeError:
                values.append(float(row))
        values = [value for value in values if value >= 0.0]
        if not values:
            return 0.0
        if self._matrix_reduce == "sum":
            return sum(values)
        if self._matrix_reduce == "mean":
            return sum(values) / len(values)
        return max(values)

    def _force_msg(self, data: Any) -> Float32MultiArray:
        msg = Float32MultiArray()
        arrays = data if isinstance(data, (list, tuple)) else []
        msg.data = (
            self._array5(arrays[0] if len(arrays) > 0 else [])
            + self._array5(arrays[1] if len(arrays) > 1 else [])
            + self._array5(arrays[2] if len(arrays) > 2 else [])
            + self._array5(arrays[3] if len(arrays) > 3 else [])
        )
        return msg

    @staticmethod
    def _array5(values: Any) -> list[float]:
        try:
            out = [float(value) for value in values]
        except TypeError:
            out = []
        if len(out) < 5:
            out.extend([0.0] * (5 - len(out)))
        return out[:5]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
