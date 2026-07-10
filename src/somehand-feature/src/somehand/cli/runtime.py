"""Runtime factories for CLI command handlers."""

from __future__ import annotations

import argparse

from somehand.app import (
    BiHandRetargetingEngine,
    BiHandRetargetingSession,
    ControlledRetargetingSession,
    RetargetingEngine,
    RetargetingSession,
)
from somehand.runtime import (
    AsyncBiHandLandmarkOutputSink,
    AsyncLandmarkOutputSink,
    BiHandOutputWindowSink,
    BiHandVideoOutputSink,
    LinkerHandModelAdapter,
    LinkerHandSdkController,
    MujocoSimController,
    OpenCvPreviewWindow,
    RobotHandOutputSink,
    RobotHandTargetOutputSink,
    RobotHandVideoOutputSink,
    infer_linkerhand_model_family,
)
from somehand.infrastructure.l20_tip_ik import L20ContactIKRefiner


def close_resource(resource: object) -> None:
    close_fn = getattr(resource, "close", None)
    if callable(close_fn):
        close_fn()


def _close_sinks(frame_sinks: list[object], sinks: list[object]) -> None:
    for sink in reversed(frame_sinks):
        close_resource(sink)
    for sink in reversed(sinks):
        close_resource(sink)


def _build_visual_sinks(
    engine: RetargetingEngine,
    *,
    backend: str,
    key_callback=None,
    include_landmark_viewer: bool = True,
    include_sim_state_viewer: bool = True,
) -> tuple[list[object], list[object]]:
    sinks: list[object] = []
    frame_sinks: list[object] = []
    if include_landmark_viewer:
        frame_sinks.append(AsyncLandmarkOutputSink(window_title="Input Landmarks"))
    if backend == "sim":
        sinks.append(
            RobotHandTargetOutputSink(
                engine.hand_model,
                key_callback=key_callback,
                window_title="Retargeting",
            )
        )
        if include_sim_state_viewer:
            sinks.append(
                RobotHandOutputSink(
                    engine.hand_model,
                    key_callback=key_callback,
                    window_title="Sim State",
                )
            )
    else:
        sinks.append(
            RobotHandOutputSink(
                engine.hand_model,
                key_callback=key_callback,
                window_title="Retargeting",
            )
        )
    return sinks, frame_sinks


def _build_control_visual_sinks(
    engine: RetargetingEngine,
    *,
    backend: str,
    key_callback=None,
    include_landmark_viewer: bool = True,
    include_sim_state_viewer: bool = True,
) -> tuple[list[object], list[object]]:
    sinks: list[object] = []
    frame_sinks: list[object] = []
    if include_landmark_viewer:
        frame_sinks.append(AsyncLandmarkOutputSink(window_title="Input Landmarks"))
    if backend == "sim":
        sinks.append(
            RobotHandTargetOutputSink(
                engine.hand_model,
                key_callback=key_callback,
                window_title="Retargeting",
            )
        )
        if include_sim_state_viewer:
            sinks.append(
                RobotHandOutputSink(
                    engine.hand_model,
                    key_callback=key_callback,
                    window_title="Sim State",
                )
            )
    elif backend == "real":
        sinks.append(
            RobotHandTargetOutputSink(
                engine.hand_model,
                key_callback=key_callback,
                window_title="Retargeting",
            )
        )
    else:
        sinks.append(
            RobotHandOutputSink(
                engine.hand_model,
                key_callback=key_callback,
                window_title="Retargeting",
            )
        )
    return sinks, frame_sinks


def _append_video_sink(
    sinks: list[object],
    *,
    hand_model,
    video_output_path: str | None,
    video_output_fps: int | None,
) -> None:
    if video_output_path is None:
        return
    if video_output_fps is None:
        raise ValueError("video_output_fps is required when video_output_path is provided")
    sinks.append(
        RobotHandVideoOutputSink(
            hand_model,
            output_path=video_output_path,
            fps=video_output_fps,
        )
    )


def _build_bihand_visual_sinks(
    engine: BiHandRetargetingEngine,
    *,
    key_callback=None,
) -> tuple[list[object], list[object]]:
    frame_sinks = [
        AsyncBiHandLandmarkOutputSink(
            left_pos=engine.config.viewer.left_pos,
            right_pos=engine.config.viewer.right_pos,
            left_quat=engine.config.viewer.left_quat,
            right_quat=engine.config.viewer.right_quat,
        )
    ]
    sinks = [
        BiHandOutputWindowSink(
            engine.left_engine.hand_model,
            engine.right_engine.hand_model,
            key_callback=key_callback,
            panel_width=engine.config.viewer.panel_width,
            panel_height=engine.config.viewer.panel_height,
            window_name=engine.config.viewer.window_name,
            left_pos=engine.config.viewer.left_pos,
            right_pos=engine.config.viewer.right_pos,
            camera_lookat=engine.config.viewer.camera_lookat,
            left_quat=engine.config.viewer.left_quat,
            right_quat=engine.config.viewer.right_quat,
        )
    ]
    return sinks, frame_sinks


def build_engine(args: argparse.Namespace, *, input_type: str) -> RetargetingEngine:
    return RetargetingEngine.from_config_path(args.config, input_type=input_type)


def build_bihand_engine(args: argparse.Namespace, *, input_type: str) -> BiHandRetargetingEngine:
    return BiHandRetargetingEngine.from_config_path(args.config, input_type=input_type)


def build_session(
    engine: RetargetingEngine,
    *,
    backend: str = "viewer",
    visualize: bool,
    show_preview: bool,
    key_callback=None,
    video_output_path: str | None = None,
    video_output_fps: int | None = None,
) -> RetargetingSession:
    sinks: list[object] = []
    frame_sinks: list[object] = []
    if visualize:
        try:
            sinks, frame_sinks = _build_visual_sinks(
                engine,
                backend=backend,
                key_callback=key_callback,
            )
        except BaseException:
            _close_sinks(frame_sinks, sinks)
            raise
    _append_video_sink(
        sinks,
        hand_model=engine.hand_model,
        video_output_path=video_output_path,
        video_output_fps=video_output_fps,
    )
    preview_window = OpenCvPreviewWindow() if show_preview else None
    return RetargetingSession(engine, sinks=sinks, frame_sinks=frame_sinks, preview_window=preview_window)


def build_control_backend(args: argparse.Namespace, engine: RetargetingEngine):
    if args.backend == "sim":
        return MujocoSimController(
            engine.config.hand.mjcf_path,
            control_rate_hz=args.control_rate,
            sim_rate_hz=args.sim_rate,
        )
    if args.backend == "real":
        family = args.model_family or engine.config.controller.model_family or infer_linkerhand_model_family(
            engine.config.hand.name
        )
        adapter = LinkerHandModelAdapter(
            engine.hand_model,
            family=family,
            hand_side=engine.config.hand.side,
            sdk_root="" if args.sdk_root is None else args.sdk_root,
        )
        return LinkerHandSdkController(
            adapter,
            transport=args.transport,
            can_interface=args.can_interface,
            modbus_port=args.modbus_port,
            default_speed=engine.config.controller.default_speed or adapter.default_speed,
            default_torque=engine.config.controller.default_torque or adapter.default_torque,
            sdk_root="" if args.sdk_root is None else args.sdk_root,
            real_state_print_every=args.real_state_print_every,
            real_command_print_every=args.real_command_print_every,
            real_can_dump=args.real_can_dump,
            l20_flexion_calibration=args.l20_flexion_calibration,
            l20_flexion_deadzone=args.l20_flexion_deadzone,
            l20_flexion_base_follow_tip=args.l20_flexion_base_follow_tip,
            l20_flexion_tip_gamma=args.l20_flexion_tip_gamma,
            l20_flexion_close_command_floor=args.l20_flexion_close_command_floor,
            l20_invert_finger_yaw=args.l20_invert_finger_yaw,
            l20_joint_range_mapping=args.l20_joint_range_mapping,
        )
    raise ValueError(f"Unsupported backend: {args.backend}")


def build_l20_contact_ik_refiner(args: argparse.Namespace, engine: RetargetingEngine) -> L20ContactIKRefiner | None:
    if not getattr(args, "l20_contact_ik", False):
        return None
    family = args.model_family or engine.config.controller.model_family or infer_linkerhand_model_family(
        engine.config.hand.name
    )
    if family.upper() != "L20":
        raise ValueError("--l20-contact-ik currently only supports model family L20")
    if engine.config.hand.side != "right":
        raise ValueError("--l20-contact-ik currently only supports the right L20 hand")
    return L20ContactIKRefiner(
        engine.hand_model,
        hand_side=engine.config.hand.side,
        active_joints=args.l20_contact_active_joints,
        active_contacts=args.l20_contact_active_contacts,
        objective=args.l20_contact_objective,
        max_delta_from_base=args.l20_contact_max_delta,
        contact_weights=args.l20_contact_weights,
        contact_target_scales=args.l20_contact_target_scales,
        contact_min_target_distances=args.l20_contact_min_target_distances,
        soft_pitch_delta=args.l20_contact_soft_pitch_delta,
    )


def build_runtime_session(
    engine: RetargetingEngine,
    args: argparse.Namespace,
    *,
    visualize: bool,
    show_preview: bool,
    key_callback=None,
    video_output_path: str | None = None,
    video_output_fps: int | None = None,
    include_landmark_viewer: bool = True,
    include_sim_state_viewer: bool = True,
):
    if args.backend == "viewer":
        return build_session(
            engine,
            backend=args.backend,
            visualize=visualize,
            show_preview=show_preview,
            key_callback=key_callback,
            video_output_path=video_output_path,
            video_output_fps=video_output_fps,
        )

    sinks: list[object] = []
    frame_sinks: list[object] = []
    if visualize:
        try:
            sinks, frame_sinks = _build_control_visual_sinks(
                engine,
                backend=args.backend,
                key_callback=key_callback,
                include_landmark_viewer=include_landmark_viewer,
                include_sim_state_viewer=include_sim_state_viewer,
            )
        except BaseException:
            _close_sinks(frame_sinks, sinks)
            raise
    _append_video_sink(
        sinks,
        hand_model=engine.hand_model,
        video_output_path=video_output_path,
        video_output_fps=video_output_fps,
    )
    preview_window = OpenCvPreviewWindow() if show_preview else None
    controller = build_control_backend(args, engine)
    contact_ik_refiner = build_l20_contact_ik_refiner(args, engine)
    return ControlledRetargetingSession(
        engine,
        controller,
        sinks=sinks,
        frame_sinks=frame_sinks,
        preview_window=preview_window,
        mediapipe_angle_print_every=getattr(args, "mediapipe_angle_print_every", 0),
        l20_contact_ik_refiner=contact_ik_refiner,
        l20_contact_ik_print_every=getattr(args, "l20_contact_ik_print_every", 0),
    )


def build_bihand_session(
    engine: BiHandRetargetingEngine,
    *,
    visualize: bool,
    show_preview: bool,
    key_callback=None,
    video_output_path: str | None = None,
    video_output_fps: int | None = None,
) -> BiHandRetargetingSession:
    sinks: list[object] = []
    frame_sinks: list[object] = []
    if visualize:
        try:
            sinks, frame_sinks = _build_bihand_visual_sinks(engine, key_callback=key_callback)
        except BaseException:
            _close_sinks(frame_sinks, sinks)
            raise
    if video_output_path is not None:
        if video_output_fps is None:
            raise ValueError("video_output_fps is required when video_output_path is provided")
        sinks.append(
            BiHandVideoOutputSink(
                engine.left_engine.hand_model,
                engine.right_engine.hand_model,
                output_path=video_output_path,
                fps=video_output_fps,
                panel_width=engine.config.viewer.panel_width,
                panel_height=engine.config.viewer.panel_height,
                left_pos=engine.config.viewer.left_pos,
                right_pos=engine.config.viewer.right_pos,
                camera_lookat=engine.config.viewer.camera_lookat,
                left_quat=engine.config.viewer.left_quat,
                right_quat=engine.config.viewer.right_quat,
            )
        )
    preview_window = OpenCvPreviewWindow("Bi-Hand Detection") if show_preview else None
    return BiHandRetargetingSession(engine, sinks=sinks, frame_sinks=frame_sinks, preview_window=preview_window)


__all__ = [
    "build_bihand_engine",
    "build_bihand_session",
    "build_control_backend",
    "build_engine",
    "build_l20_contact_ik_refiner",
    "build_runtime_session",
    "build_session",
    "close_resource",
]
