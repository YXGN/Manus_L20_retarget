"""Live source adapters for MediaPipe, hc_mocap, and PICO."""

from __future__ import annotations

import numpy as np

from somehand.core import BiHandFrame, BiHandSourceFrame, HandFrame, SourceFrame, normalize_hand_side
from somehand.domain.hand_detection import HandDetection
from somehand.hc_mocap_input import _DirectHCMocapUDPProvider, create_hc_mocap_udp_provider, hc_mocap_frame_to_landmarks
from somehand.pico_input import PicoBridgeReceiver, create_pico_provider, pico_frame_to_detection

from .source_transforms import annotate_bihand_preview, annotate_preview, copy_bihand_frame, to_bihand_frame, to_hand_frame

HandDetector = None


def _load_hand_detector_cls():
    global HandDetector
    if HandDetector is None:
        from somehand.hand_detector import HandDetector as detector_cls

        HandDetector = detector_cls
    return HandDetector


class MediaPipeInputSource:
    def __init__(
        self,
        source: int | str,
        *,
        hand_side: str | None,
        swap_handedness: bool,
        source_desc: str,
        min_detection_confidence: float = 0.7,
        min_tracking_confidence: float = 0.5,
        camera_width: int | None = None,
        camera_height: int | None = None,
        camera_fps: int | None = None,
        tracking_hold_frames: int = 0,
        tracking_jump_threshold: float = 0.0,
    ):
        detector_cls = _load_hand_detector_cls()
        self.source_desc = source_desc
        self.hand_side = None if hand_side is None else normalize_hand_side(hand_side)
        self._frames = detector_cls.create_source(
            source,
            width=camera_width,
            height=camera_height,
            fps=camera_fps,
        )
        self._detector = detector_cls(
            target_hand=self.hand_side,
            swap_handedness=swap_handedness,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._available = True
        self._hold_frames = max(0, int(tracking_hold_frames))
        self._jump_threshold = max(0.0, float(tracking_jump_threshold))
        self._last_detection: HandFrame | None = None
        self._missed_frames = 0

    @property
    def fps(self) -> int:
        return 30

    def is_available(self) -> bool:
        return self._available

    def get_frame(self) -> SourceFrame:
        if not self._available:
            raise StopIteration
        try:
            preview_frame = next(self._frames)
        except StopIteration as exc:
            self._available = False
            raise StopIteration from exc

        detection = self._stabilize_detection(self._detector.detect(preview_frame))
        return SourceFrame(
            detection=None if detection is None else to_hand_frame(detection),
            preview_frame=preview_frame,
        )

    def annotate_preview(self, frame: np.ndarray, detection: HandFrame) -> np.ndarray:
        return annotate_preview(frame, detection)

    def reset(self) -> bool:
        return False

    def close(self) -> None:
        self._detector.close()

    def stats_snapshot(self) -> dict[str, object]:
        return {}

    def _stabilize_detection(self, detection: HandDetection | None) -> HandDetection | None:
        if detection is None:
            self._missed_frames += 1
            if self._last_detection is not None and self._missed_frames <= self._hold_frames:
                return HandDetection(
                    landmarks_3d=np.array(self._last_detection.landmarks_3d, copy=True),
                    landmarks_2d=np.array(self._last_detection.landmarks_2d, copy=True),
                    hand_side=self._last_detection.hand_side,
                )
            return None

        current = to_hand_frame(detection)
        if (
            self._jump_threshold > 0.0
            and self._last_detection is not None
            and self._is_landmark_jump(current, self._last_detection)
        ):
            self._missed_frames += 1
            if self._missed_frames <= self._hold_frames:
                return HandDetection(
                    landmarks_3d=np.array(self._last_detection.landmarks_3d, copy=True),
                    landmarks_2d=np.array(self._last_detection.landmarks_2d, copy=True),
                    hand_side=self._last_detection.hand_side,
                )
            return None

        self._last_detection = current
        self._missed_frames = 0
        return detection

    def _is_landmark_jump(self, current: HandFrame, previous: HandFrame) -> bool:
        if current.landmarks_3d.shape != previous.landmarks_3d.shape:
            return True
        wrist_delta = float(np.linalg.norm(current.landmarks_3d[0] - previous.landmarks_3d[0]))
        centered_current = current.landmarks_3d - current.landmarks_3d[0:1]
        centered_previous = previous.landmarks_3d - previous.landmarks_3d[0:1]
        shape_delta = float(np.median(np.linalg.norm(centered_current - centered_previous, axis=1)))
        return max(wrist_delta, shape_delta) > self._jump_threshold


class RealSenseMediaPipeInputSource:
    def __init__(
        self,
        *,
        hand_side: str | None,
        swap_handedness: bool,
        source_desc: str,
        min_detection_confidence: float = 0.7,
        min_tracking_confidence: float = 0.5,
        color_width: int = 1280,
        color_height: int = 720,
        depth_width: int = 1280,
        depth_height: int = 720,
        fps: int = 30,
        depth_patch_radius: int = 2,
        depth_scale: float = 1.0,
    ):
        detector_cls = _load_hand_detector_cls()
        self.source_desc = source_desc
        self.hand_side = None if hand_side is None else normalize_hand_side(hand_side)
        self._detector = detector_cls(
            target_hand=self.hand_side,
            swap_handedness=swap_handedness,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._pipeline = None
        self._align = None
        self._profile = None
        self._intrinsics = None
        self._rs = None
        self._available = True
        self._fps = int(fps)
        self._depth_patch_radius = max(0, int(depth_patch_radius))
        self._depth_scale = float(depth_scale)
        self._start_pipeline(
            color_width=int(color_width),
            color_height=int(color_height),
            depth_width=int(depth_width),
            depth_height=int(depth_height),
            fps=int(fps),
        )

    @property
    def fps(self) -> int:
        return self._fps

    def is_available(self) -> bool:
        return self._available

    def get_frame(self) -> SourceFrame:
        if not self._available:
            raise StopIteration

        frames = self._pipeline.wait_for_frames()
        aligned_frames = self._align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return SourceFrame(detection=None)

        color_bgr = np.asanyarray(color_frame.get_data())
        detection = self._detector.detect(color_bgr)
        if detection is None:
            return SourceFrame(detection=None, preview_frame=color_bgr)

        landmarks_3d = self._deproject_detection(detection, depth_frame)
        real_detection = HandDetection(
            landmarks_3d=landmarks_3d,
            landmarks_2d=np.array(detection.landmarks_2d, copy=True),
            hand_side=detection.hand_side,
        )
        return SourceFrame(detection=to_hand_frame(real_detection), preview_frame=color_bgr)

    def annotate_preview(self, frame: np.ndarray, detection: HandFrame) -> np.ndarray:
        return annotate_preview(frame, detection)

    def reset(self) -> bool:
        return False

    def close(self) -> None:
        self._detector.close()
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None

    def stats_snapshot(self) -> dict[str, object]:
        return {
            "fps": self._fps,
            "depth_patch_radius": self._depth_patch_radius,
            "depth_scale": self._depth_scale,
        }

    def _start_pipeline(
        self,
        *,
        color_width: int,
        color_height: int,
        depth_width: int,
        depth_height: int,
        fps: int,
    ) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "RealSense input requires pyrealsense2. Install it in the somehand conda env, "
                "for example: pip install pyrealsense2"
            ) from exc

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, fps)
        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()

        self._rs = rs
        self._pipeline = pipeline
        self._align = align
        self._profile = profile
        self._intrinsics = color_profile.get_intrinsics()

    def _deproject_detection(self, detection: HandDetection, depth_frame) -> np.ndarray:
        points = []
        for pixel in detection.landmarks_2d:
            x = int(round(float(pixel[0])))
            y = int(round(float(pixel[1])))
            depth_m = self._depth_at(depth_frame, x, y)
            point = self._rs.rs2_deproject_pixel_to_point(
                self._intrinsics,
                [float(pixel[0]), float(pixel[1])],
                depth_m,
            )
            points.append(point)

        landmarks = np.asarray(points, dtype=np.float64) * self._depth_scale
        if not np.all(np.isfinite(landmarks)):
            return np.array(detection.landmarks_3d, copy=True)
        return landmarks

    def _depth_at(self, depth_frame, x: int, y: int) -> float:
        width = int(depth_frame.get_width())
        height = int(depth_frame.get_height())
        radius = self._depth_patch_radius
        depths: list[float] = []
        for yy in range(max(0, y - radius), min(height, y + radius + 1)):
            for xx in range(max(0, x - radius), min(width, x + radius + 1)):
                depth = float(depth_frame.get_distance(xx, yy))
                if depth > 0.0:
                    depths.append(depth)
        if depths:
            return float(np.median(depths))
        return float(depth_frame.get_distance(min(max(x, 0), width - 1), min(max(y, 0), height - 1)))


class BiHandMediaPipeInputSource:
    def __init__(
        self,
        source: int | str,
        *,
        swap_handedness: bool,
        source_desc: str,
        min_detection_confidence: float = 0.7,
        min_tracking_confidence: float = 0.5,
        camera_width: int | None = None,
        camera_height: int | None = None,
        camera_fps: int | None = None,
    ):
        detector_cls = _load_hand_detector_cls()
        self.source_desc = source_desc
        self._frames = detector_cls.create_source(
            source,
            width=camera_width,
            height=camera_height,
            fps=camera_fps,
        )
        self._detector = detector_cls(
            num_hands=2,
            target_hand=None,
            swap_handedness=swap_handedness,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._available = True
        self._latest_frame: BiHandFrame | None = None
        self._frame_index = 0

    @property
    def fps(self) -> int:
        return 30

    def is_available(self) -> bool:
        return self._available

    def get_frame(self) -> BiHandSourceFrame:
        if not self._available:
            raise StopIteration
        try:
            preview_frame = next(self._frames)
        except StopIteration as exc:
            self._available = False
            raise StopIteration from exc

        detections = self._detector.detect_all(preview_frame) or []
        left_detection = next((item for item in detections if item.hand_side == "left"), None)
        right_detection = next((item for item in detections if item.hand_side == "right"), None)
        detection = to_bihand_frame(left=left_detection, right=right_detection)
        self._frame_index += 1
        self._latest_frame = detection if detection.has_detection else None
        return BiHandSourceFrame(
            detection=detection if detection.has_detection else None,
            preview_frame=preview_frame,
        )

    def latest_bihand_frame_snapshot(self) -> tuple[int, BiHandFrame] | None:
        if self._latest_frame is None:
            return None
        return self._frame_index, copy_bihand_frame(self._latest_frame)

    def annotate_preview(self, frame: np.ndarray, detection: BiHandFrame) -> np.ndarray:
        return annotate_bihand_preview(frame, detection)

    def reset(self) -> bool:
        return False

    def close(self) -> None:
        self._detector.close()

    def stats_snapshot(self) -> dict[str, object]:
        return {}


class HCMocapInputSource:
    def __init__(self, provider: object, *, source_desc: str):
        self.source_desc = source_desc
        self._provider = provider

    @property
    def fps(self) -> int:
        return int(getattr(self._provider, "fps", 30))

    def is_available(self) -> bool:
        return bool(self._provider.is_available())

    def get_frame(self) -> SourceFrame:
        detection = self._provider.get_detection()
        return SourceFrame(
            detection=to_hand_frame(detection),
        )

    def latest_hand_frame_snapshot(self) -> tuple[int, HandFrame] | None:
        snapshot_fn = getattr(self._provider, "latest_detection_snapshot", None)
        if not callable(snapshot_fn):
            return None

        snapshot = snapshot_fn()
        if snapshot is None:
            return None

        frame_index, detection = snapshot
        return frame_index, to_hand_frame(detection)

    def reset(self) -> bool:
        reset_fn = getattr(getattr(self._provider, "_provider", None), "reset", None)
        if not callable(reset_fn):
            return False
        reset_fn()
        return True

    def close(self) -> None:
        self._provider.close()

    def stats_snapshot(self) -> dict[str, object]:
        stats_fn = getattr(self._provider, "stats_snapshot", None)
        if callable(stats_fn):
            return dict(stats_fn())
        return {}


class BiHandPicoInputSource:
    def __init__(
        self,
        *,
        timeout: float,
        host: str = "0.0.0.0",
        port: int = 63901,
        discovery: bool = True,
        advertise_ip: str | None = None,
    ):
        self.source_desc = "pico://both"
        self._timeout = float(timeout)
        self._receiver = PicoBridgeReceiver(
            host=host,
            port=port,
            discovery=discovery,
            advertise_ip=advertise_ip,
            timeout=timeout,
        )
        self._last_served_seq = 0

    @property
    def fps(self) -> int:
        return self._receiver.fps

    def is_available(self) -> bool:
        return self._receiver.is_available()

    def get_frame(self) -> BiHandSourceFrame:
        frame = self._receiver.wait_frame(
            timeout=self._timeout,
            after_seq=self._last_served_seq if self._last_served_seq > 0 else None,
        )
        self._last_served_seq = int(getattr(frame, "seq", self._last_served_seq + 1))
        detection = to_bihand_frame(
            left=pico_frame_to_detection(frame, "left"),
            right=pico_frame_to_detection(frame, "right"),
        )
        return BiHandSourceFrame(detection=detection if detection.has_detection else None)

    def latest_bihand_frame_snapshot(self) -> tuple[int, BiHandFrame] | None:
        frame = self._receiver.latest_frame()
        if frame is None:
            return None
        detection = to_bihand_frame(
            left=pico_frame_to_detection(frame, "left"),
            right=pico_frame_to_detection(frame, "right"),
        )
        if not detection.has_detection:
            return None
        return int(getattr(frame, "seq", 0)), detection

    def reset(self) -> bool:
        return False

    def close(self) -> None:
        self._receiver.close()

    def stats_snapshot(self) -> dict[str, object]:
        return self._receiver.stats_snapshot()


class BiHCMocapInputSource:
    def __init__(
        self,
        *,
        reference_bvh: str | None,
        host: str,
        port: int,
        timeout: float,
    ):
        self._provider = _DirectHCMocapUDPProvider(
            reference_bvh=reference_bvh,
            host=host,
            port=port,
            timeout=timeout,
        )
        self.source_desc = f"udp://{host or '0.0.0.0'}:{port}"

    @property
    def fps(self) -> int:
        return self._provider.fps

    def is_available(self) -> bool:
        return self._provider.is_available()

    def get_frame(self) -> BiHandSourceFrame:
        frame = self._provider.get_frame()
        return BiHandSourceFrame(detection=self._frame_to_detection(frame))

    def latest_bihand_frame_snapshot(self) -> tuple[int, BiHandFrame] | None:
        snapshot = self._provider.latest_frame_snapshot()
        if snapshot is None:
            return None
        frame_index, frame = snapshot
        return frame_index, self._frame_to_detection(frame)

    def reset(self) -> bool:
        return False

    def close(self) -> None:
        self._provider.close()

    def stats_snapshot(self) -> dict[str, object]:
        return dict(self._provider.stats_snapshot())

    @staticmethod
    def _frame_to_detection(frame: dict[str, tuple[np.ndarray, np.ndarray]]) -> BiHandFrame:
        left = HandDetection(
            landmarks_3d=hc_mocap_frame_to_landmarks(frame, "left"),
            landmarks_2d=np.zeros((21, 2), dtype=np.float64),
            hand_side="left",
        )
        right = HandDetection(
            landmarks_3d=hc_mocap_frame_to_landmarks(frame, "right"),
            landmarks_2d=np.zeros((21, 2), dtype=np.float64),
            hand_side="right",
        )
        return to_bihand_frame(left=left, right=right)


def create_hc_mocap_udp_source(
    *,
    reference_bvh: str | None,
    hand_side: str,
    host: str,
    port: int,
    timeout: float,
) -> HCMocapInputSource:
    provider = create_hc_mocap_udp_provider(
        reference_bvh=reference_bvh,
        hand_side=hand_side,
        host=host,
        port=port,
        timeout=timeout,
    )
    return HCMocapInputSource(
        provider,
        source_desc=f"udp://{host or '0.0.0.0'}:{port}",
    )


def create_pico_source(
    *,
    hand_side: str,
    timeout: float,
    host: str = "0.0.0.0",
    port: int = 63901,
    discovery: bool = True,
    advertise_ip: str | None = None,
) -> HCMocapInputSource:
    normalized_side = normalize_hand_side(hand_side)
    provider = create_pico_provider(
        hand_side=normalized_side,
        timeout=timeout,
        host=host,
        port=port,
        discovery=discovery,
        advertise_ip=advertise_ip,
    )
    return HCMocapInputSource(
        provider,
        source_desc=f"pico://{normalized_side}",
    )


def create_bihand_pico_source(
    *,
    timeout: float,
    host: str = "0.0.0.0",
    port: int = 63901,
    discovery: bool = True,
    advertise_ip: str | None = None,
) -> BiHandPicoInputSource:
    return BiHandPicoInputSource(
        timeout=timeout,
        host=host,
        port=port,
        discovery=discovery,
        advertise_ip=advertise_ip,
    )


def create_bihand_hc_mocap_udp_source(
    *,
    reference_bvh: str | None,
    host: str,
    port: int,
    timeout: float,
) -> BiHCMocapInputSource:
    return BiHCMocapInputSource(
        reference_bvh=reference_bvh,
        host=host,
        port=port,
        timeout=timeout,
    )
