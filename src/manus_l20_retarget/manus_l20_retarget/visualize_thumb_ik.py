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
from .manus_somehand_retarget_node import (
    DEFAULT_FINGER_YAW_OPEN_RAD,
    DEFAULT_ROOT_CLOSED_RAD,
    DEFAULT_ROOT_OPEN_RAD,
    DEFAULT_TIP_CLOSED_RAD,
    DEFAULT_TIP_OPEN_RAD,
    FINGER_LANDMARKS,
    STANDARD_FIST_COMMAND,
    STANDARD_OPEN_COMMAND,
    THUMB_COMMAND_SLOTS,
    _ThumbLocalIK,
    _ThumbPointMapper,
    _default_workspace_root,
    _joint_flexion_rad,
    _lerp_command,
    _load_thumb_calibration_samples,
    _manus_thumb_local_point,
    _normalized_angle,
)
from .mapping import clamp_u8
from .manus_landmarks import _finger_yaw_rad, _palm_frame


IDENTITY_MAT = np.eye(3, dtype=np.float64).reshape(-1)
BLUE = np.asarray([0.1, 0.35, 1.0, 1.0], dtype=np.float32)
GREEN = np.asarray([0.1, 0.9, 0.25, 1.0], dtype=np.float32)
RED = np.asarray([1.0, 0.15, 0.1, 1.0], dtype=np.float32)
YELLOW = np.asarray([1.0, 0.8, 0.1, 1.0], dtype=np.float32)


class ThumbIKVisualizer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("manus_thumb_ik_visualizer")
        self._args = args
        self._last_print_time = 0.0
        self._open_command = list(STANDARD_OPEN_COMMAND)
        self._closed_command = list(STANDARD_FIST_COMMAND)
        self._root_open_rad = list(DEFAULT_ROOT_OPEN_RAD)
        self._root_closed_rad = list(DEFAULT_ROOT_CLOSED_RAD)
        self._tip_open_rad = list(DEFAULT_TIP_OPEN_RAD)
        self._tip_closed_rad = list(DEFAULT_TIP_CLOSED_RAD)
        self._thumb_tip_closed_command = int(STANDARD_FIST_COMMAND[15])
        self._thumb_tip_with_root_closed_command = int(STANDARD_FIST_COMMAND[15])
        self._load_flexion_calibration(args.flexion_calibration_path)
        self._load_somehand()
        self._viewer = self._launch_viewer()
        self.create_subscription(ManusGlove, args.topic, self._on_glove, 10)
        self.get_logger().info(
            "thumb IK visualizer started: blue=origin, green=target, red=actual tip, yellow=residual"
        )

    def _load_flexion_calibration(self, path_value: str) -> None:
        path_text = str(path_value).strip()
        if not path_text:
            return
        path = Path(path_text).expanduser().resolve()
        if not path.exists():
            self.get_logger().warning(f"flexion calibration not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load flexion calibration {path}: {exc}")
            return
        self._root_open_rad = _float_list(data.get("root_flexion_open_rad"), self._root_open_rad, 5)
        self._root_closed_rad = _float_list(data.get("root_flexion_closed_rad"), self._root_closed_rad, 5)
        self._tip_open_rad = _float_list(data.get("tip_flexion_open_rad"), self._tip_open_rad, 5)
        self._tip_closed_rad = _float_list(data.get("tip_flexion_closed_rad"), self._tip_closed_rad, 5)

        command_data = data.get("command")
        if isinstance(command_data, dict):
            open_command = _command_list(command_data.get("open_command"), self._open_command)
            four_closed = _command_list(command_data.get("four_finger_closed_command"), self._closed_command)
            thumb_closed = _command_list(command_data.get("thumb_closed_command"), self._closed_command)
            thumb_tip_closed = _command_list(command_data.get("thumb_tip_closed_command"), thumb_closed)
            for slot in (0, 1, 2, 3, 4, 15, 16, 17, 18, 19):
                self._open_command[slot] = open_command[slot]
            for slot in (1, 2, 3, 4, 16, 17, 18, 19):
                self._closed_command[slot] = four_closed[slot]
            self._closed_command[0] = thumb_closed[0]
            self._closed_command[15] = thumb_tip_closed[15]
            self._thumb_tip_closed_command = int(thumb_tip_closed[15])
            self._thumb_tip_with_root_closed_command = int(thumb_closed[15])
        self.get_logger().info(
            "loaded visual flexion calibration "
            f"path={path}, thumb_root_closed={self._closed_command[0]}, "
            f"thumb_tip_only_closed={self._thumb_tip_closed_command}, "
            f"thumb_tip_with_root_closed={self._thumb_tip_with_root_closed_command}"
        )

    def _load_somehand(self) -> None:
        somehand_root = Path(self._args.somehand_root).expanduser().resolve()
        src_path = somehand_root / "src"
        if str(src_path) not in sys.path:
            sys.path.insert(0, str(src_path))

        from somehand.api import RetargetingEngine
        from somehand.infrastructure.controllers.adapters import LinkerHandModelAdapter

        self._engine = RetargetingEngine.from_config_path(str(Path(self._args.config).expanduser()), input_type="landmarks")
        self._adapter = LinkerHandModelAdapter(
            self._engine.hand_model,
            family="L20",
            hand_side=self._engine.config.hand.side,
            sdk_root=str(Path(self._args.linkerhand_sdk_root).expanduser()),
        )
        self._thumb_ik = _ThumbLocalIK(self._engine.hand_model)
        samples = _load_thumb_calibration_samples(self._args.thumb_calibration_path)
        self._point_mapper = _ThumbPointMapper(samples, adapter=self._adapter, thumb_ik=self._thumb_ik)
        self._init_segment_solver()

    def _init_segment_solver(self) -> None:
        self._segment_body_id = self._thumb_ik._resolve_body(self._args.robot_segment_body)
        joint_index = self._engine.hand_model.get_joint_name_to_qpos_index()
        self._segment_joint_names = tuple(name for name in ("thumb_cmc_yaw", "thumb_cmc_roll") if name in joint_index)
        self._segment_qpos_ids = np.asarray([joint_index[name] for name in self._segment_joint_names], dtype=np.int32)
        self._segment_dof_ids = self._thumb_ik._qpos_ids_to_dof_ids(self._segment_qpos_ids)
        self._segment_lower, self._segment_upper = self._thumb_ik._active_joint_ranges(self._segment_qpos_ids)
        open_qpos = self._adapter.sdk_range_to_qpos(list(self._open_command))
        self._engine.hand_model.set_qpos(open_qpos.copy())
        self._robot_open_segment_vector = self._robot_segment_vector().copy()
        self._robot_open_segment_unit = _unit_vector(self._robot_open_segment_vector)
        self._segment_length = max(float(np.linalg.norm(self._robot_open_segment_vector)), 1e-6)
        self._segment_open_samples: list[np.ndarray] = []
        self._segment_open_start_time: float | None = None
        self._segment_open_rotation: np.ndarray | None = None
        self._last_segment_qpos: np.ndarray | None = None

    def _launch_viewer(self) -> Any:
        from somehand.runtime.viewer_camera import configure_free_camera
        from somehand.runtime.viewer_passive import ManagedPassiveViewer, set_viewer_overlay_label

        viewer = ManagedPassiveViewer(
            self._engine.hand_model.model,
            self._engine.hand_model.data,
            window_title="L20 Thumb IK Debug",
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
            "blue origin | green mapped target | red actual tip | yellow residual",
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
        manus_point = _manus_thumb_local_point(landmarks)
        base_command = self._four_finger_command(landmarks)
        if self._args.mode == "point":
            self._apply_thumb_flexion(base_command, landmarks)
            qpos = self._adapter.sdk_range_to_qpos(base_command)
            point = self._manus_thumb_local_point_at(landmarks, self._args.point_index)
            target_vector = self._point_mapper.map(point) if self._args.point_map_mode == "mapped" else point
            progress = 0.0
            yaw_bias_qpos = None
        elif self._args.mode == "segment":
            self._apply_thumb_flexion(base_command, landmarks)
            base_qpos = self._adapter.sdk_range_to_qpos(base_command)
            target_vector = self._thumb_segment_target_vector(landmarks)
            qpos = self._solve_segment_yaw_roll(target_vector, base_qpos)
            progress = 0.0
            yaw_bias_qpos = None
        else:
            target_vector = self._point_mapper.map(manus_point)
            base_qpos = self._adapter.sdk_range_to_qpos(base_command)
            progress = self._point_mapper.open_progress(manus_point)
            yaw_bias_qpos = None
            if self._args.yaw_bias:
                yaw_bias_qpos = float(self._args.yaw_bias_min + progress * (self._args.yaw_bias_max - self._args.yaw_bias_min))
            qpos = self._thumb_ik.solve_target(
                target_vector,
                base_qpos,
                yaw_bias_qpos=yaw_bias_qpos,
                yaw_bias_weight=self._args.yaw_bias_weight if yaw_bias_qpos is not None else 0.0,
            )
        command = [int(value) for value in self._adapter.qpos_to_sdk_range(qpos)]

        model = self._engine.hand_model.model
        data = self._engine.hand_model.data
        origin = data.xpos[self._thumb_ik.roll_body_id].copy()
        target = origin + target_vector
        actual = data.site_xpos[self._thumb_ik.tip_site_id].copy()
        residual = float(np.linalg.norm(actual - target))

        with self._viewer.lock():
            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            origin = data.xpos[self._thumb_ik.roll_body_id].copy()
            target = origin + target_vector
            actual = (
                data.xpos[self._segment_body_id].copy()
                if self._args.mode in ("segment", "point")
                else data.site_xpos[self._thumb_ik.tip_site_id].copy()
            )
            residual = float(np.linalg.norm(actual - target))
            self._draw_overlay(self._viewer.user_scn, origin, target, actual)
        self._viewer.sync()

        now = time.monotonic()
        if now - self._last_print_time >= self._args.interval:
            self._last_print_time = now
            self.get_logger().info(
                "thumb_ik_visual "
                f"manus_point={np.round(manus_point, 5).tolist()} "
                f"target_vec={np.round(target_vector, 5).tolist()} "
                f"origin={np.round(origin, 5).tolist()} "
                f"target={np.round(target, 5).tolist()} "
                f"actual_point={np.round(actual, 5).tolist()} "
                f"residual={residual:.5f} "
                f"mode={self._args.mode} "
                f"align={self._segment_align_status()} "
                f"progress={progress:.3f} "
                f"yaw_bias={None if yaw_bias_qpos is None else round(yaw_bias_qpos, 5)} "
                f"four={[base_command[index] for index in range(1, 5)] + [base_command[index] for index in range(6, 10)] + [base_command[index] for index in range(16, 20)]} "
                f"cmd={[command[index] for index in THUMB_COMMAND_SLOTS]}"
            )

    def _segment_align_status(self) -> str:
        if self._args.mode != "segment" or not self._args.segment_align_open:
            return "off"
        return "done" if self._segment_open_rotation is not None else "calibrating"

    def _apply_thumb_flexion(self, command: list[int], landmarks: np.ndarray) -> None:
        root_angle = _joint_flexion_rad(landmarks[1], landmarks[2], landmarks[3])
        tip_angle = _joint_flexion_rad(landmarks[2], landmarks[3], landmarks[4])
        tip_amount = _normalized_angle(
            tip_angle,
            self._tip_open_rad[0],
            self._tip_closed_rad[0],
            self._args.thumb_tip_gamma,
        )
        root_amount = _normalized_angle(
            root_angle,
            self._root_open_rad[0],
            self._root_closed_rad[0],
            self._args.thumb_root_gamma,
        )
        effective_tip_closed_command = _lerp_command(
            self._thumb_tip_closed_command,
            self._thumb_tip_with_root_closed_command,
            root_amount,
        )
        command[0] = _lerp_command(self._open_command[0], self._closed_command[0], root_amount)
        command[15] = _lerp_command(self._open_command[15], effective_tip_closed_command, tip_amount)
        command[5] = self._open_command[5]
        command[10] = self._open_command[10]

    def _thumb_segment_target_vector(self, landmarks: np.ndarray) -> np.ndarray:
        start = self._manus_thumb_local_point_at(landmarks, self._args.segment_start)
        end = self._manus_thumb_local_point_at(landmarks, self._args.segment_end)
        if self._args.segment_start == self._args.segment_end:
            return self._point_mapper.map(end) if self._args.segment_map_mode == "mapped" else end
        if self._args.segment_map_mode == "mapped":
            vector = self._point_mapper.map(end) - self._point_mapper.map(start)
        else:
            vector = end - start
        vector = self._align_segment_open(vector)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-8:
            return self._robot_segment_vector()
        return vector / norm * self._segment_length * self._args.segment_scale

    def _align_segment_open(self, vector: np.ndarray) -> np.ndarray:
        if not self._args.segment_align_open:
            return np.asarray(vector, dtype=np.float64)
        vector = np.asarray(vector, dtype=np.float64)
        unit = _unit_vector(vector)
        if unit is None:
            return vector
        if self._segment_open_rotation is not None:
            return self._segment_open_rotation @ vector

        now = time.monotonic()
        if self._segment_open_start_time is None:
            self._segment_open_start_time = now
        self._segment_open_samples.append(unit)
        if now - self._segment_open_start_time < self._args.segment_open_calibration_sec:
            return self._robot_open_segment_vector.copy()

        open_unit = _unit_vector(np.mean(np.asarray(self._segment_open_samples, dtype=np.float64), axis=0))
        if open_unit is None or self._robot_open_segment_unit is None:
            self._segment_open_rotation = np.eye(3, dtype=np.float64)
        else:
            self._segment_open_rotation = _rotation_between(open_unit, self._robot_open_segment_unit)
        return self._segment_open_rotation @ vector

    def _manus_thumb_local_point_at(self, landmarks: np.ndarray, index: int) -> np.ndarray:
        frame = _palm_frame(landmarks)
        vector = np.asarray(landmarks[index] - landmarks[1], dtype=np.float64)
        if frame is None:
            return vector
        lateral, forward, normal = frame
        return np.asarray(
            [
                float(np.dot(vector, lateral)),
                float(np.dot(vector, forward)),
                float(np.dot(vector, normal)),
            ],
            dtype=np.float64,
        )

    def _solve_segment_yaw_roll(self, target_vector: np.ndarray, base_qpos: np.ndarray) -> np.ndarray:
        qpos = np.asarray(base_qpos, dtype=np.float64).copy()
        if self._last_segment_qpos is not None:
            qpos[self._segment_qpos_ids] = self._last_segment_qpos[self._segment_qpos_ids]
        target = np.asarray(target_vector, dtype=np.float64)
        for _ in range(36):
            self._engine.hand_model.set_qpos(qpos)
            residual = self._robot_segment_vector() - target
            if float(np.linalg.norm(residual)) <= 8e-4:
                break
            jacobian = self._robot_segment_jacobian()
            lhs = jacobian.T @ jacobian
            rhs = -(jacobian.T @ residual)
            lhs += self._args.segment_damping * np.eye(len(self._segment_qpos_ids), dtype=np.float64)
            try:
                delta = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            norm = float(np.linalg.norm(delta))
            if norm > self._args.segment_max_step:
                delta *= self._args.segment_max_step / max(norm, 1e-8)
            active = np.clip(qpos[self._segment_qpos_ids] + delta, self._segment_lower, self._segment_upper)
            qpos[self._segment_qpos_ids] = active
            self._engine.hand_model.apply_mimic_constraints(qpos)
        self._engine.hand_model.set_qpos(qpos)
        self._last_segment_qpos = qpos.copy()
        return qpos

    def _robot_segment_vector(self) -> np.ndarray:
        data = self._engine.hand_model.data
        return data.xpos[self._segment_body_id].copy() - data.xpos[self._thumb_ik.roll_body_id].copy()

    def _robot_segment_jacobian(self) -> np.ndarray:
        model = self._engine.hand_model.model
        data = self._engine.hand_model.data
        jac_segment = np.zeros((3, model.nv), dtype=np.float64)
        jac_origin = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacBody(model, data, jac_segment, None, int(self._segment_body_id))
        mujoco.mj_jacBody(model, data, jac_origin, None, int(self._thumb_ik.roll_body_id))
        return (jac_segment - jac_origin)[:, self._segment_dof_ids]

    def _four_finger_command(self, landmarks: np.ndarray) -> list[int]:
        command = list(self._open_command)
        for finger_index, (mcp, pip, dip, tip) in enumerate(FINGER_LANDMARKS):
            if finger_index == 0:
                continue
            root_angle = _joint_flexion_rad(landmarks[mcp], landmarks[pip], landmarks[dip])
            tip_angle = _joint_flexion_rad(landmarks[pip], landmarks[dip], landmarks[tip])
            root_amount = _normalized_angle(
                root_angle,
                self._root_open_rad[finger_index],
                self._root_closed_rad[finger_index],
                1.0,
            )
            tip_amount = _normalized_angle(
                tip_angle,
                self._tip_open_rad[finger_index],
                self._tip_closed_rad[finger_index],
                1.0,
            )
            command[finger_index] = _lerp_command(
                self._open_command[finger_index],
                self._closed_command[finger_index],
                root_amount,
            )
            command[15 + finger_index] = _lerp_command(
                self._open_command[15 + finger_index],
                self._closed_command[15 + finger_index],
                tip_amount,
            )

        yaw_angles = _finger_yaw_rad(landmarks, source="tip")
        for local_index, angle in enumerate(yaw_angles):
            slot = 6 + local_index
            delta = -500.0 * (float(angle) - DEFAULT_FINGER_YAW_OPEN_RAD[local_index])
            delta = max(-120.0, min(120.0, delta))
            command[slot] = clamp_u8(self._open_command[slot] + delta)

        for index in range(11, 15):
            command[index] = 255
        return command

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
    somehand_root = root / "src" / "somehand-feature"
    retarget_config = root / "src" / "manus_l20_retarget" / "config"
    parser = argparse.ArgumentParser(description="Visualize MANUS thumb point mapping and L20 thumb IK in MuJoCo.")
    parser.add_argument("--topic", default="/manus_glove_0")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--mode", choices=("segment", "tip", "point"), default="segment")
    parser.add_argument("--transform", default="pico_native_to_rh")
    parser.add_argument("--wrist-mode", default="estimate", choices=("estimate", "palm_center"))
    parser.add_argument("--distal-mode", default="dip", choices=("dip", "ip"))
    parser.add_argument("--somehand-root", default=str(somehand_root))
    parser.add_argument(
        "--config",
        default=str(somehand_root / "configs" / "retargeting" / "right" / "linkerhand_l20_right.yaml"),
    )
    parser.add_argument(
        "--linkerhand-sdk-root",
        default=str(somehand_root / "third_party" / "linkerhand-python-sdk"),
    )
    parser.add_argument(
        "--thumb-calibration-path",
        default="",
    )
    parser.add_argument(
        "--flexion-calibration-path",
        default=str(retarget_config / "flexion_right_calibration.yaml"),
    )
    parser.add_argument("--segment-start", type=int, default=2)
    parser.add_argument("--segment-end", type=int, default=3)
    parser.add_argument("--segment-map-mode", choices=("mapped", "raw"), default="mapped")
    parser.add_argument("--point-index", type=int, default=2)
    parser.add_argument("--point-map-mode", choices=("mapped", "raw"), default="mapped")
    parser.add_argument("--segment-align-open", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--segment-open-calibration-sec", type=float, default=1.0)
    parser.add_argument("--segment-scale", type=float, default=1.0)
    parser.add_argument("--segment-damping", type=float, default=8e-4)
    parser.add_argument("--segment-max-step", type=float, default=0.20)
    parser.add_argument("--robot-segment-body", default="thumb_metacarpals")
    parser.add_argument("--thumb-root-gamma", type=float, default=1.0)
    parser.add_argument("--thumb-tip-gamma", type=float, default=1.0)
    parser.add_argument("--yaw-bias", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--yaw-bias-min", type=float, default=0.0)
    parser.add_argument("--yaw-bias-max", type=float, default=1.25)
    parser.add_argument("--yaw-bias-weight", type=float, default=0.045)
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args()
    rclpy.init(args=args)
    node = ThumbIKVisualizer(parsed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _unit_vector(vector: np.ndarray) -> np.ndarray | None:
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-8:
        return None
    return vector / norm


def _rotation_between(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    source = source / max(float(np.linalg.norm(source)), 1e-8)
    target = target / max(float(np.linalg.norm(target)), 1e-8)
    cross = np.cross(source, target)
    dot = float(np.dot(source, target))
    if dot > 1.0 - 1e-8:
        return np.eye(3, dtype=np.float64)
    if dot < -1.0 + 1e-8:
        axis = np.cross(source, np.asarray([1.0, 0.0, 0.0], dtype=np.float64))
        if float(np.linalg.norm(axis)) <= 1e-8:
            axis = np.cross(source, np.asarray([0.0, 1.0, 0.0], dtype=np.float64))
        axis = axis / max(float(np.linalg.norm(axis)), 1e-8)
        return _axis_angle_rotation(axis, np.pi)
    skew = np.asarray(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + skew + skew @ skew * (1.0 / (1.0 + dot))


def _axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-8)
    x, y, z = axis
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    one_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def _float_list(value: Any, fallback: list[float], length: int) -> list[float]:
    if not isinstance(value, (list, tuple)):
        return list(fallback)
    result = list(fallback)
    for index, item in enumerate(value[:length]):
        try:
            result[index] = float(item)
        except (TypeError, ValueError):
            pass
    return result


def _command_list(value: Any, fallback: list[int]) -> list[int]:
    if not isinstance(value, (list, tuple)):
        return list(fallback)
    result = list(fallback)
    for index, item in enumerate(value[: len(result)]):
        try:
            result[index] = clamp_u8(item)
        except (TypeError, ValueError):
            pass
    return result


if __name__ == "__main__":
    main()
