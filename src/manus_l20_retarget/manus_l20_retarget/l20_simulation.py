from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import rclpy
import yaml
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node

from .manus_landmarks import manus_raw_nodes_to_mediapipe_landmarks
from .manus_l20_retarget_node import (
    DEFAULT_ROOT_CLOSED_RAD,
    DEFAULT_ROOT_OPEN_RAD,
    DEFAULT_TIP_CLOSED_RAD,
    DEFAULT_TIP_OPEN_RAD,
    FINGER_LANDMARKS,
    STANDARD_FIST_COMMAND,
    STANDARD_OPEN_COMMAND,
    THUMB_COMMAND_SLOTS,
    THUMB_IK_COMMAND_SLOTS,
    _ThumbLocalIK,
    _ThumbSegmentIK,
    _command_parameter,
    _default_workspace_root,
    _float_list_parameter,
    _frame_rotation_from_two_vectors,
    _lerp_command,
    _manus_thumb_local_point_at,
    _normalized_angle,
    _normalized_signed_segment,
    _optional_vector3_parameter,
    _rotation_between,
    _scale_command_delta,
    _scale_command_delta_ease_in_with_deadzone,
    _unit_vector,
)
from .mapping import clamp_u8
from .retarget_pipeline import _directed_finger_flexion_rad, _joint_flexion_rad


IDENTITY_MAT = np.eye(3, dtype=np.float64).reshape(-1)
BLUE = np.asarray([0.1, 0.35, 1.0, 1.0], dtype=np.float32)
GREEN = np.asarray([0.1, 0.9, 0.25, 1.0], dtype=np.float32)
RED = np.asarray([1.0, 0.15, 0.1, 1.0], dtype=np.float32)
YELLOW = np.asarray([1.0, 0.8, 0.1, 1.0], dtype=np.float32)


class L20SimulationNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("manus_l20_simulation")
        self._args = args
        self._last_print_time = 0.0

        self._neutral_command = list(STANDARD_OPEN_COMMAND)
        self._closed_command = list(STANDARD_FIST_COMMAND)
        self._root_open_rad = list(DEFAULT_ROOT_OPEN_RAD)
        self._root_closed_rad = list(DEFAULT_ROOT_CLOSED_RAD)
        self._tip_open_rad = list(DEFAULT_TIP_OPEN_RAD)
        self._tip_closed_rad = list(DEFAULT_TIP_CLOSED_RAD)
        self._root_flexion_direction_sign: list[float] | None = None
        self._tip_flexion_direction_sign: list[float] | None = None
        self._finger_yaw_mapping: dict[str, Any] | None = None
        self._thumb_segment_robot_open_command = _command_parameter(
            args.thumb_segment_robot_open_command,
            STANDARD_OPEN_COMMAND,
        )
        self._thumb_flexion_mapping: dict[str, float | int] | None = None
        self._segment_open_samples: list[np.ndarray] = []
        self._segment_open_start_time: float | None = None
        self._segment_open_rotation: np.ndarray | None = None
        self._segment_frame_data = self._load_segment_frame(args.segment_frame_path)
        self._segment_manus_open_vector = self._load_segment_open_vector(
            args.segment_manus_open_vector,
            args.segment_manus_open_vector_path,
        )

        self._load_flexion_calibration(args.flexion_calibration_path)
        self._load_finger_yaw_calibration(args.finger_yaw_calibration_path)
        self._load_thumb_flexion_mapping(args.thumb_flexion_mapping_path)
        self._load_l20_thumb_ik()
        self._viewer = self._launch_viewer()
        self.create_subscription(ManusGlove, args.topic, self._on_glove, 10)
        self.get_logger().info(
            "MANUS -> L20 MuJoCo simulation started: "
            f"input={args.topic}, thumb_debug={args.thumb_debug}"
        )
        if args.thumb_debug:
            self.get_logger().info(
                "thumb debug overlay: blue=L20 segment origin, green=MANUS mapped target, "
                "red=L20 solved segment end, yellow=residual"
            )

    def _load_flexion_calibration(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            return
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            self.get_logger().warning(f"visual flexion calibration not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load visual flexion calibration {path}: {exc}")
            return

        self._root_open_rad = _float_list_parameter(
            data.get("root_flexion_open_rad"),
            self._root_open_rad,
            length=5,
        )
        self._root_closed_rad = _float_list_parameter(
            data.get("root_flexion_closed_rad"),
            self._root_closed_rad,
            length=5,
        )
        self._tip_open_rad = _float_list_parameter(
            data.get("tip_flexion_open_rad"),
            self._tip_open_rad,
            length=5,
        )
        self._tip_closed_rad = _float_list_parameter(
            data.get("tip_flexion_closed_rad"),
            self._tip_closed_rad,
            length=5,
        )
        samples = data.get("samples")
        open_sample = samples.get("open") if isinstance(samples, dict) else None
        fist_sample = samples.get("four_finger_fist") if isinstance(samples, dict) else None
        has_signed_samples = (
            isinstance(open_sample, dict)
            and isinstance(fist_sample, dict)
            and "root_signed_rad" in open_sample
            and "root_signed_rad" in fist_sample
            and "tip_signed_rad" in open_sample
            and "tip_signed_rad" in fist_sample
        )
        self._root_flexion_direction_sign = (
            _float_list_parameter(data.get("root_flexion_direction_sign"), [1.0] * 5, length=5)
            if has_signed_samples and data.get("root_flexion_direction_sign") is not None
            else None
        )
        self._tip_flexion_direction_sign = (
            _float_list_parameter(data.get("tip_flexion_direction_sign"), [1.0] * 5, length=5)
            if has_signed_samples and data.get("tip_flexion_direction_sign") is not None
            else None
        )

        command_data = data.get("command")
        if isinstance(command_data, dict):
            open_command = _command_parameter(command_data.get("open_command"), self._neutral_command)
            four_closed = _command_parameter(
                command_data.get("four_finger_closed_command"),
                self._closed_command,
            )
            for slot in (1, 2, 3, 4, 16, 17, 18, 19):
                self._neutral_command[slot] = open_command[slot]
                self._closed_command[slot] = four_closed[slot]

        self.get_logger().info(
            "loaded visual flexion calibration "
            f"path={path}, root_open={np.round(self._root_open_rad, 5).tolist()}, "
            f"root_closed={np.round(self._root_closed_rad, 5).tolist()}, "
            f"tip_open={np.round(self._tip_open_rad, 5).tolist()}, "
            f"tip_closed={np.round(self._tip_closed_rad, 5).tolist()}"
        )

    def _load_finger_yaw_calibration(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            return
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            self.get_logger().warning(f"visual finger yaw calibration not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load visual finger yaw calibration {path}: {exc}")
            return

        samples = data.get("samples")
        command_data = data.get("command")
        if not isinstance(samples, dict) or not isinstance(command_data, dict):
            self.get_logger().warning(f"invalid visual finger yaw calibration file: {path}")
            return
        open_sample = samples.get("natural_open")
        close_sample = samples.get("finger_close")
        spread_sample = samples.get("finger_spread")
        if not all(isinstance(sample, dict) for sample in (open_sample, close_sample, spread_sample)):
            self.get_logger().warning(f"visual finger yaw calibration missing required samples: {path}")
            return

        open_command = _command_parameter(command_data.get("natural_open_command"), self._neutral_command)
        close_command = _command_parameter(command_data.get("finger_close_command"), self._neutral_command)
        spread_command = _command_parameter(command_data.get("finger_spread_command"), self._neutral_command)
        source = str(data.get("source", "")).strip().lower()
        if source != "ergonomics":
            self.get_logger().warning(
                f"visual finger yaw source={source!r} is no longer supported; "
                "use an ergonomics yaw calibration file"
            )
            return
        ergonomics_keys = data.get("ergonomics_keys")
        if not isinstance(ergonomics_keys, list) or len(ergonomics_keys) != 4:
            self.get_logger().warning(f"visual finger yaw ergonomics calibration missing four ergonomics_keys: {path}")
            return
        ergonomics_keys = [str(key) for key in ergonomics_keys]
        try:
            self._finger_yaw_mapping = {
                "source": source,
                "ergonomics_keys": ergonomics_keys,
                "open_rad": _float_list_parameter(open_sample.get("yaw_rad"), [0.0] * 4, length=4),
                "close_rad": _float_list_parameter(close_sample.get("yaw_rad"), [0.0] * 4, length=4),
                "spread_rad": _float_list_parameter(spread_sample.get("yaw_rad"), [0.0] * 4, length=4),
                "open_cmd": [open_command[slot] for slot in range(6, 10)],
                "close_cmd": [close_command[slot] for slot in range(6, 10)],
                "spread_cmd": [spread_command[slot] for slot in range(6, 10)],
            }
        except (TypeError, ValueError, IndexError) as exc:
            self.get_logger().warning(f"invalid visual finger yaw calibration values in {path}: {exc}")
            self._finger_yaw_mapping = None
            return
        self.get_logger().info(
            "loaded visual finger yaw calibration "
            f"path={path}, source={source}, "
            f"ergonomics_keys={ergonomics_keys}, "
            f"open_cmd={self._finger_yaw_mapping['open_cmd']}, "
            f"close_cmd={self._finger_yaw_mapping['close_cmd']}, "
            f"spread_cmd={self._finger_yaw_mapping['spread_cmd']}"
        )

    def _load_thumb_flexion_mapping(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            return
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            self.get_logger().warning(f"thumb flexion mapping not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load thumb flexion mapping {path}: {exc}")
            return

        samples = data.get("samples")
        command_data = data.get("command")
        if not isinstance(samples, dict) or not isinstance(command_data, dict):
            self.get_logger().warning(f"invalid thumb flexion mapping file: {path}")
            return
        open_sample = samples.get("thumb_natural_open")
        touch_sample = samples.get("thumb_pinky_root_touch")
        if not isinstance(open_sample, dict) or not isinstance(touch_sample, dict):
            self.get_logger().warning(f"thumb flexion mapping missing required samples: {path}")
            return

        open_command = _command_parameter(command_data.get("thumb_natural_open_command"), self._neutral_command)
        touch_command = _command_parameter(command_data.get("thumb_pinky_root_touch_command"), self._neutral_command)
        try:
            self._thumb_flexion_mapping = {
                "root_open_rad": float(open_sample["root_rad"]),
                "root_touch_rad": float(touch_sample["root_rad"]),
                "tip_open_rad": float(open_sample["tip_rad"]),
                "tip_touch_rad": float(touch_sample["tip_rad"]),
                "root_direction_sign": (
                    float(data["root_flexion_direction_sign"])
                    if data.get("root_flexion_direction_sign") is not None
                    else None
                ),
                "tip_direction_sign": (
                    float(data["tip_flexion_direction_sign"])
                    if data.get("tip_flexion_direction_sign") is not None
                    else None
                ),
                "root_open_cmd": int(open_command[0]),
                "root_touch_cmd": int(touch_command[0]),
                "tip_open_cmd": int(open_command[15]),
                "tip_touch_cmd": int(touch_command[15]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid thumb flexion mapping values in {path}: {exc}")
            self._thumb_flexion_mapping = None
            return
        self.get_logger().info(
            "loaded visual thumb flexion mapping "
            f"path={path}, "
            f"root_rad=[{self._thumb_flexion_mapping['root_open_rad']:.5f}, "
            f"{self._thumb_flexion_mapping['root_touch_rad']:.5f}], "
            f"tip_rad=[{self._thumb_flexion_mapping['tip_open_rad']:.5f}, "
            f"{self._thumb_flexion_mapping['tip_touch_rad']:.5f}], "
            f"root_cmd=[{self._thumb_flexion_mapping['root_open_cmd']}, "
            f"{self._thumb_flexion_mapping['root_touch_cmd']}], "
            f"tip_cmd=[{self._thumb_flexion_mapping['tip_open_cmd']}, "
            f"{self._thumb_flexion_mapping['tip_touch_cmd']}]"
        )

    def _load_segment_open_vector(self, inline_value: str, path_value: str) -> np.ndarray | None:
        inline_vector = _optional_vector3_parameter(inline_value)
        if inline_vector is not None:
            self.get_logger().info(
                "using inline MANUS thumb segment open vector "
                f"{np.round(inline_vector, 6).tolist()}"
            )
            return inline_vector

        path_text = str(path_value).strip()
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        if not path.exists():
            self.get_logger().info(
                f"thumb segment open vector file not found: {path}; "
                "visualizer will use startup open alignment"
            )
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            vector = _optional_vector3_parameter(data.get("manus_open_vector"))
            if vector is None:
                raise ValueError("missing manus_open_vector")
            segment_start = data.get("segment_start")
            segment_end = data.get("segment_end")
            if segment_start is not None and int(segment_start) != self._args.segment_start:
                self.get_logger().warning(
                    f"open vector segment_start={segment_start} does not match "
                    f"--segment-start={self._args.segment_start}"
                )
            if segment_end is not None and int(segment_end) != self._args.segment_end:
                self.get_logger().warning(
                    f"open vector segment_end={segment_end} does not match "
                    f"--segment-end={self._args.segment_end}"
                )
            self.get_logger().info(
                f"loaded MANUS thumb segment open vector path={path}, "
                f"vector={np.round(vector, 6).tolist()}"
            )
            return vector
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(
                f"failed to load thumb segment open vector from {path}: {exc}; "
                "visualizer will use startup open alignment"
            )
            return None

    def _load_segment_frame(self, path_value: str) -> dict[str, Any] | None:
        path_text = str(path_value).strip()
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        if not path.exists():
            self.get_logger().info(
                f"thumb segment frame file not found: {path}; visualizer will use open-vector alignment"
            )
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            open_vector = _optional_vector3_parameter(data.get("manus_open_vector"))
            touch_vector = _optional_vector3_parameter(data.get("manus_touch_vector"))
            if open_vector is None or touch_vector is None:
                raise ValueError("missing manus_open_vector/manus_touch_vector")
            return {
                "path": path,
                "manus_open_vector": open_vector,
                "manus_touch_vector": touch_vector,
                "robot_open_command": _command_parameter(data.get("robot_open_command"), self._thumb_segment_robot_open_command),
                "robot_touch_command": _command_parameter(data.get("robot_touch_command"), self._thumb_segment_robot_open_command),
            }
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(
                f"failed to load thumb segment frame from {path}: {exc}; visualizer will use open-vector alignment"
            )
            return None

    def _load_l20_thumb_ik(self) -> None:
        thumb_ik_root = Path(self._args.l20_thumb_ik_root).expanduser().resolve()
        src_path = thumb_ik_root / "src"
        if str(src_path) not in sys.path:
            sys.path.insert(0, str(src_path))

        from l20_ik_core.api import RetargetingEngine
        from l20_ik_core.infrastructure.controllers.adapters import LinkerHandModelAdapter

        self._engine = RetargetingEngine.from_config_path(str(Path(self._args.config).expanduser()), input_type="landmarks")
        self._adapter = LinkerHandModelAdapter(
            self._engine.hand_model,
            family="L20",
            hand_side=self._engine.config.hand.side,
            sdk_root=str(Path(self._args.linkerhand_sdk_root).expanduser()),
        )
        self._thumb_local_ik = _ThumbLocalIK(self._engine.hand_model)
        self._thumb_segment_ik = _ThumbSegmentIK(
            self._engine.hand_model,
            adapter=self._adapter,
            thumb_ik=self._thumb_local_ik,
            robot_segment_body=self._args.robot_segment_body,
            robot_open_command=self._thumb_segment_robot_open_command,
        )
        self._initialize_segment_frame_rotation()
        self.get_logger().info(
            f"loaded L20 model config={Path(self._args.config).expanduser()}, "
            f"side={self._engine.config.hand.side}, "
            f"robot_open_thumb={[self._thumb_segment_robot_open_command[index] for index in THUMB_COMMAND_SLOTS]}"
        )

    def _initialize_segment_frame_rotation(self) -> None:
        if self._segment_frame_data is None:
            return
        data = self._segment_frame_data
        manus_open = np.asarray(data["manus_open_vector"], dtype=np.float64)
        manus_touch = np.asarray(data["manus_touch_vector"], dtype=np.float64)
        robot_open = self._robot_segment_vector_for_command(data["robot_open_command"])
        robot_touch = self._robot_segment_vector_for_command(data["robot_touch_command"])
        rotation = _frame_rotation_from_two_vectors(manus_open, manus_touch, robot_open, robot_touch)
        if rotation is None:
            self.get_logger().warning("thumb segment frame is degenerate; visualizer will use open-vector alignment")
            self._segment_frame_data = None
            return
        self._segment_open_rotation = rotation
        self.get_logger().info(
            f"initialized visual thumb segment frame path={data['path']}, "
            f"robot_open={np.round(robot_open, 6).tolist()}, "
            f"robot_touch={np.round(robot_touch, 6).tolist()}"
        )

    def _robot_segment_vector_for_command(self, command: list[int]) -> np.ndarray:
        qpos = self._adapter.sdk_range_to_qpos(command)
        self._thumb_segment_ik.hand_model.set_qpos(qpos.copy())
        return self._thumb_segment_ik.current_segment_vector()

    def _launch_viewer(self) -> Any:
        from l20_ik_core.runtime.viewer_camera import configure_free_camera
        from l20_ik_core.runtime.viewer_passive import ManagedPassiveViewer, set_viewer_overlay_label

        viewer = ManagedPassiveViewer(
            self._engine.hand_model.model,
            self._engine.hand_model.data,
            window_title="MANUS L20 MuJoCo Simulation",
            show_left_ui=False,
            show_right_ui=False,
        )
        with viewer.lock():
            configure_free_camera(
                viewer.cam,
                distance=0.45,
                azimuth=135.0,
                elevation=-25.0,
                lookat=(0.02, 0.06, 0.03),
            )
        set_viewer_overlay_label(
            viewer,
            "MANUS -> L20 simulation | use --thumb-debug for segment IK overlay",
        )
        viewer.sync(state_only=True)
        return viewer

    def _on_glove(self, msg: ManusGlove) -> None:
        if not self._viewer.is_running():
            rclpy.shutdown()
            return

        landmarks = manus_raw_nodes_to_mediapipe_landmarks(
            msg.raw_nodes,
            transform=self._args.transform,
            wrist_mode=self._args.wrist_mode,
            distal_mode=self._args.distal_mode,
        )
        ergonomics = {str(entry.type): float(entry.value) for entry in msg.ergonomics if entry.type}
        base_command = self._base_command(landmarks, ergonomics)
        base_qpos = self._adapter.sdk_range_to_qpos(base_command)
        target_vector = self._thumb_segment_target_vector(landmarks)
        raw_qpos = self._thumb_segment_ik.solve_target(
            target_vector,
            base_qpos,
            damping=self._args.segment_damping,
            max_step=self._args.segment_max_step,
        )
        raw_command = [clamp_u8(value) for value in self._adapter.qpos_to_sdk_range(raw_qpos)]
        command = list(raw_command)
        command[5] = _scale_command_delta_ease_in_with_deadzone(
            command[5],
            self._thumb_segment_robot_open_command[5],
            self._args.roll_command_scale,
            self._args.roll_command_gamma,
            self._args.roll_command_deadzone,
        )
        command[10] = _scale_command_delta(
            command[10],
            self._thumb_segment_robot_open_command[10],
            self._args.yaw_command_scale,
        )
        final_command = command
        final_qpos = self._adapter.sdk_range_to_qpos(final_command)

        model = self._engine.hand_model.model
        data = self._engine.hand_model.data
        with self._viewer.lock():
            data.qpos[:] = final_qpos
            mujoco.mj_forward(model, data)
            origin = data.xpos[self._thumb_segment_ik.origin_body_id].copy()
            target = origin + target_vector
            actual = data.xpos[self._thumb_segment_ik.segment_body_id].copy()
            residual = float(np.linalg.norm(actual - target))
            if self._args.thumb_debug:
                self._draw_overlay(self._viewer.user_scn, origin, target, actual)
            else:
                self._viewer.user_scn.ngeom = 0
        self._viewer.sync()

        now = time.monotonic()
        if now - self._last_print_time >= self._args.interval:
            self._last_print_time = now
            start = _manus_thumb_local_point_at(landmarks, self._args.segment_start)
            end = _manus_thumb_local_point_at(landmarks, self._args.segment_end)
            self.get_logger().info(
                "thumb_segment_visual "
                f"start={self._args.segment_start} end={self._args.segment_end} "
                f"align={self._segment_align_status()} "
                f"manus_start={np.round(start, 5).tolist()} "
                f"manus_end={np.round(end, 5).tolist()} "
                f"target={np.round(target_vector, 5).tolist()} "
                f"actual={np.round(actual - origin, 5).tolist()} "
                f"residual={residual:.5f} "
                f"raw_roll_yaw={[raw_command[index] for index in THUMB_IK_COMMAND_SLOTS]} "
                f"computed_thumb={[command[index] for index in THUMB_COMMAND_SLOTS]} "
                f"shown_thumb={[final_command[index] for index in THUMB_COMMAND_SLOTS]} "
                f"base_thumb={[base_command[index] for index in THUMB_COMMAND_SLOTS]}"
            )

    def _base_command(self, landmarks: np.ndarray, ergonomics: dict[str, float]) -> list[int]:
        command = list(self._neutral_command)
        if self._args.show_fingers:
            self._apply_visual_finger_flexion(command, landmarks)
            if self._args.finger_yaw:
                self._apply_visual_finger_yaw(command, ergonomics)

        mapping = self._thumb_flexion_mapping
        if mapping is None:
            return command

        root_sign = mapping["root_direction_sign"]
        tip_sign = mapping["tip_direction_sign"]
        root_angle = (
            _joint_flexion_rad(landmarks[1], landmarks[2], landmarks[3])
            if root_sign is None
            else max(0.0, _directed_finger_flexion_rad(landmarks, 0, root=True) * float(root_sign))
        )
        tip_angle = (
            _joint_flexion_rad(landmarks[2], landmarks[3], landmarks[4])
            if tip_sign is None
            else max(0.0, _directed_finger_flexion_rad(landmarks, 0, root=False) * float(tip_sign))
        )
        root_amount = _normalized_angle(
            root_angle,
            float(mapping["root_open_rad"]),
            float(mapping["root_touch_rad"]),
            self._args.thumb_root_gamma,
        )
        tip_amount = _normalized_angle(
            tip_angle,
            float(mapping["tip_open_rad"]),
            float(mapping["tip_touch_rad"]),
            self._args.thumb_tip_gamma,
        )
        command[0] = _lerp_command(int(mapping["root_open_cmd"]), int(mapping["root_touch_cmd"]), root_amount)
        command[15] = _lerp_command(int(mapping["tip_open_cmd"]), int(mapping["tip_touch_cmd"]), tip_amount)
        return command

    def _apply_visual_finger_flexion(self, command: list[int], landmarks: np.ndarray) -> None:
        for finger_index, (mcp, pip, dip, tip) in enumerate(FINGER_LANDMARKS):
            if finger_index == 0:
                continue
            if self._root_flexion_direction_sign is None:
                root_angle = _joint_flexion_rad(landmarks[mcp], landmarks[pip], landmarks[dip])
            else:
                root_angle = max(
                    0.0,
                    _directed_finger_flexion_rad(landmarks, finger_index, root=True)
                    * float(self._root_flexion_direction_sign[finger_index]),
                )
            if self._tip_flexion_direction_sign is None:
                tip_angle = _joint_flexion_rad(landmarks[pip], landmarks[dip], landmarks[tip])
            else:
                tip_angle = max(
                    0.0,
                    _directed_finger_flexion_rad(landmarks, finger_index, root=False)
                    * float(self._tip_flexion_direction_sign[finger_index]),
                )
            root_amount = _normalized_angle(
                root_angle,
                self._root_open_rad[finger_index],
                self._root_closed_rad[finger_index],
                self._args.root_gamma,
            )
            tip_amount = _normalized_angle(
                tip_angle,
                self._tip_open_rad[finger_index],
                self._tip_closed_rad[finger_index],
                self._args.tip_gamma,
            )
            command[finger_index] = _lerp_command(
                self._neutral_command[finger_index],
                self._closed_command[finger_index],
                root_amount,
            )
            command[15 + finger_index] = _lerp_command(
                self._neutral_command[15 + finger_index],
                self._closed_command[15 + finger_index],
                tip_amount,
            )

    def _apply_visual_finger_yaw(
        self,
        command: list[int],
        ergonomics: dict[str, float],
    ) -> None:
        mapping = self._finger_yaw_mapping
        if mapping is None:
            return
        keys = mapping.get("ergonomics_keys") or []
        if len(keys) != 4:
            return
        try:
            yaw_angles = np.asarray([float(ergonomics[str(key)]) for key in keys], dtype=np.float64)
        except (KeyError, TypeError, ValueError):
            return

        open_rad = mapping["open_rad"]
        close_rad = mapping["close_rad"]
        spread_rad = mapping["spread_rad"]
        open_cmd = mapping["open_cmd"]
        close_cmd = mapping["close_cmd"]
        spread_cmd = mapping["spread_cmd"]
        for local_index, angle in enumerate(yaw_angles):
            slot = 6 + local_index
            close_amount = _normalized_signed_segment(
                float(angle),
                float(open_rad[local_index]),
                float(close_rad[local_index]),
            )
            spread_amount = _normalized_signed_segment(
                float(angle),
                float(open_rad[local_index]),
                float(spread_rad[local_index]),
            )
            if close_amount >= spread_amount:
                command[slot] = _lerp_command(
                    int(open_cmd[local_index]),
                    int(close_cmd[local_index]),
                    close_amount,
                )
            else:
                command[slot] = _lerp_command(
                    int(open_cmd[local_index]),
                    int(spread_cmd[local_index]),
                    spread_amount,
                )

    def _thumb_segment_target_vector(self, landmarks: np.ndarray) -> np.ndarray:
        start = _manus_thumb_local_point_at(landmarks, self._args.segment_start)
        end = _manus_thumb_local_point_at(landmarks, self._args.segment_end)
        vector = end if self._args.segment_start == self._args.segment_end else end - start
        vector = self._align_segment_open(vector)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-8:
            return self._thumb_segment_ik.robot_open_segment_vector.copy()
        return vector / norm * self._thumb_segment_ik.segment_length * self._args.segment_scale

    def _align_segment_open(self, vector: np.ndarray) -> np.ndarray:
        vector = np.asarray(vector, dtype=np.float64)
        if not self._args.segment_align_open:
            return vector
        unit = _unit_vector(vector)
        if unit is None:
            return vector
        if self._segment_open_rotation is not None:
            return self._segment_open_rotation @ vector

        if self._segment_manus_open_vector is not None:
            open_unit = _unit_vector(self._segment_manus_open_vector)
            robot_open_unit = _unit_vector(self._thumb_segment_ik.robot_open_segment_vector)
            if open_unit is None or robot_open_unit is None:
                self._segment_open_rotation = np.eye(3, dtype=np.float64)
            else:
                self._segment_open_rotation = _rotation_between(open_unit, robot_open_unit)
            return self._segment_open_rotation @ vector

        now = time.monotonic()
        if self._segment_open_start_time is None:
            self._segment_open_start_time = now
        self._segment_open_samples.append(unit)
        if now - self._segment_open_start_time < self._args.segment_open_calibration_sec:
            return self._thumb_segment_ik.robot_open_segment_vector.copy()

        open_unit = _unit_vector(np.mean(np.asarray(self._segment_open_samples, dtype=np.float64), axis=0))
        robot_open_unit = _unit_vector(self._thumb_segment_ik.robot_open_segment_vector)
        if open_unit is None or robot_open_unit is None:
            self._segment_open_rotation = np.eye(3, dtype=np.float64)
        else:
            self._segment_open_rotation = _rotation_between(open_unit, robot_open_unit)
        return self._segment_open_rotation @ vector

    def _segment_align_status(self) -> str:
        if not self._args.segment_align_open:
            return "off"
        if self._segment_frame_data is not None:
            return "frame" if self._segment_open_rotation is not None else "frame_pending"
        if self._segment_manus_open_vector is not None:
            return "fixed" if self._segment_open_rotation is not None else "fixed_pending"
        return "done" if self._segment_open_rotation is not None else "calibrating"

    def _draw_overlay(self, scene: Any, origin: np.ndarray, target: np.ndarray, actual: np.ndarray) -> None:
        scene.ngeom = 0
        self._append_sphere(scene, origin, 0.006, BLUE)
        self._append_sphere(scene, target, 0.007, GREEN)
        self._append_sphere(scene, actual, 0.007, RED)
        self._append_capsule(scene, target, actual, 0.002, YELLOW)

    def _append_sphere(self, scene: Any, point: np.ndarray, radius: float, rgba: np.ndarray) -> None:
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.full(3, float(radius), dtype=np.float64),
            np.asarray(point, dtype=np.float64),
            IDENTITY_MAT,
            rgba,
        )
        scene.ngeom += 1

    def _append_capsule(self, scene: Any, start: np.ndarray, end: np.ndarray, radius: float, rgba: np.ndarray) -> None:
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            IDENTITY_MAT,
            rgba,
        )
        mujoco.mjv_connector(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            float(radius),
            np.asarray(start, dtype=np.float64),
            np.asarray(end, dtype=np.float64),
        )
        geom.rgba[:] = rgba
        scene.ngeom += 1

    def destroy_node(self) -> bool:
        if hasattr(self, "_viewer") and self._viewer.is_running():
            self._viewer.close()
        return super().destroy_node()


def _parse_args() -> argparse.Namespace:
    root = _default_workspace_root()
    thumb_ik_root = root / "src" / "l20_thumb_ik"
    retarget_config = root / "src" / "manus_l20_retarget" / "config"
    parser = argparse.ArgumentParser(description="Run MANUS -> L20 MuJoCo simulation without real hardware.")
    parser.add_argument("--topic", default="/manus_glove_0")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--mode", choices=("segment",), default="segment", help=argparse.SUPPRESS)
    parser.add_argument("--transform", default="right_glove_to_right_retarget")
    parser.add_argument("--wrist-mode", default="estimate", choices=("estimate", "palm_center"))
    parser.add_argument("--distal-mode", default="dip", choices=("dip", "ip"))
    parser.add_argument("--l20-thumb-ik-root", default=str(thumb_ik_root))
    parser.add_argument(
        "--config",
        default=str(thumb_ik_root / "configs" / "retargeting" / "right" / "linkerhand_l20_right.yaml"),
    )
    parser.add_argument(
        "--linkerhand-sdk-root",
        default=str(thumb_ik_root / "third_party" / "linkerhand-python-sdk"),
    )
    parser.add_argument("--show-fingers", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--thumb-debug", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--flexion-calibration-path", default="")
    parser.add_argument("--finger-yaw", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--finger-yaw-calibration-path", default="")
    parser.add_argument(
        "--thumb-flexion-mapping-path",
        default=str(retarget_config / "thumb_right_flexion_mapping.yaml"),
    )
    parser.add_argument("--segment-start", type=int, default=2)
    parser.add_argument("--segment-end", type=int, default=3)
    parser.add_argument("--segment-map-mode", choices=("raw", "mapped"), default="raw", help=argparse.SUPPRESS)
    parser.add_argument("--segment-align-open", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--segment-open-calibration-sec", type=float, default=1.0)
    parser.add_argument("--segment-frame-path", default=str(retarget_config / "thumb_segment_frame_right.yaml"))
    parser.add_argument("--segment-manus-open-vector", default="")
    parser.add_argument("--segment-manus-open-vector-path", default="")
    parser.add_argument("--segment-scale", type=float, default=1.0)
    parser.add_argument("--segment-damping", type=float, default=8e-4)
    parser.add_argument("--segment-max-step", type=float, default=0.20)
    parser.add_argument("--robot-segment-body", default="thumb_metacarpals")
    parser.add_argument(
        "--thumb-segment-robot-open-command",
        default="[254,248,246,249,254,128,115,111,131,188,221,255,255,255,255,254,254,254,254,254]",
    )
    parser.add_argument("--thumb-root-gamma", type=float, default=1.0)
    parser.add_argument("--thumb-tip-gamma", type=float, default=1.0)
    parser.add_argument("--root-gamma", type=float, default=1.0)
    parser.add_argument("--tip-gamma", type=float, default=1.0)
    parser.add_argument("--roll-command-scale", type=float, default=1.0)
    parser.add_argument("--yaw-command-scale", type=float, default=1.0)
    parser.add_argument("--roll-command-deadzone", type=int, default=0)
    parser.add_argument("--roll-command-gamma", type=float, default=1.0)
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = L20SimulationNode(parsed)
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
