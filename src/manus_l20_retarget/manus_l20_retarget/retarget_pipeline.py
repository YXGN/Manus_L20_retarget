from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from manus_ros2_msgs.msg import ManusGlove

from .manus_landmarks import _palm_frame, manus_raw_nodes_to_mediapipe_landmarks
from .mapping import clamp_u8


COMMAND_LENGTH = 20
FINGER_LANDMARKS = (
    (1, 2, 3, 4),
    (5, 6, 7, 8),
    (9, 10, 11, 12),
    (13, 14, 15, 16),
    (17, 18, 19, 20),
)
L20_SLOT_NAMES = [
    "Thumb Base",
    "Index Finger Base",
    "Middle Finger Base",
    "Ring Finger Base",
    "Pinky Finger Base",
    "Thumb Roll",
    "Index Finger Yaw",
    "Middle Finger Yaw",
    "Ring Finger Yaw",
    "Pinky Finger Yaw",
    "Thumb Yaw",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Thumb Tip",
    "Index Finger Tip",
    "Middle Finger Tip",
    "Ring Finger Tip",
    "Pinky Finger Tip",
]


@dataclass(slots=True)
class HandFeatures:
    """MANUS frame data normalized for the retarget pipeline."""

    landmarks: np.ndarray
    ergonomics: dict[str, float]


@dataclass(slots=True)
class FingerFlexionTarget:
    root_amount: float
    tip_amount: float


@dataclass(slots=True)
class HandRetargetTargets:
    finger_flexion: dict[int, FingerFlexionTarget]


@dataclass(slots=True)
class L20CommandAdapter:
    neutral_command: list[int]
    closed_command: list[int]
    reserved_command: int
    lock_neutral_slots: list[int]

    def command_from_targets(self, targets: HandRetargetTargets) -> list[int]:
        command = list(self.neutral_command)
        for finger_index, target in targets.finger_flexion.items():
            command[finger_index] = _lerp_command(
                self.neutral_command[finger_index],
                self.closed_command[finger_index],
                target.root_amount,
            )
            command[15 + finger_index] = _lerp_command(
                self.neutral_command[15 + finger_index],
                self.closed_command[15 + finger_index],
                target.tip_amount,
            )
        return command

    def apply_reserved_slots(self, command: list[int]) -> None:
        for index in range(11, 15):
            command[index] = self.reserved_command

    def apply_neutral_locks(self, command: list[int]) -> None:
        for index in self.lock_neutral_slots:
            command[index] = self.neutral_command[index]


def extract_hand_features(
    msg: ManusGlove,
    *,
    landmark_transform: str,
) -> HandFeatures:
    landmarks = manus_raw_nodes_to_mediapipe_landmarks(
        msg.raw_nodes,
        transform=landmark_transform,
    )
    ergonomics = {entry.type: float(entry.value) for entry in msg.ergonomics if entry.type}
    return HandFeatures(
        landmarks=landmarks,
        ergonomics=ergonomics,
    )


def compute_landmark_flexion_targets(
    landmarks: np.ndarray,
    *,
    root_open_rad: list[float],
    root_closed_rad: list[float],
    tip_open_rad: list[float],
    tip_closed_rad: list[float],
    root_direction_sign: list[float],
    tip_direction_sign: list[float],
    root_gamma: float,
    tip_gamma: float,
    open_straightness_threshold: float,
    open_angle_deadband_rad: float = 0.0,
) -> HandRetargetTargets:
    finger_targets: dict[int, FingerFlexionTarget] = {}
    for finger_index, (mcp, pip, dip, tip) in enumerate(FINGER_LANDMARKS):
        if finger_index == 0:
            continue
        root_angle = _directed_finger_flexion_rad(landmarks, finger_index, root=True) * float(
            root_direction_sign[finger_index]
        )
        root_angle = max(0.0, root_angle)
        tip_angle = _directed_finger_flexion_rad(landmarks, finger_index, root=False) * float(
            tip_direction_sign[finger_index]
        )
        tip_angle = max(0.0, tip_angle)
        root_amount = _normalized_angle(
            root_angle,
            root_open_rad[finger_index],
            root_closed_rad[finger_index],
            root_gamma,
            open_deadband_rad=open_angle_deadband_rad,
        )
        tip_amount = _normalized_angle(
            tip_angle,
            tip_open_rad[finger_index],
            tip_closed_rad[finger_index],
            tip_gamma,
            open_deadband_rad=open_angle_deadband_rad,
        )
        root_amount = _open_straightness_guard(
            root_amount,
            landmarks[mcp],
            landmarks[pip],
            landmarks[dip],
            landmarks[tip],
            open_straightness_threshold,
        )
        tip_amount = _open_straightness_guard(
            tip_amount,
            landmarks[mcp],
            landmarks[pip],
            landmarks[dip],
            landmarks[tip],
            open_straightness_threshold,
        )
        finger_targets[finger_index] = FingerFlexionTarget(
            root_amount=root_amount,
            tip_amount=tip_amount,
        )
    return HandRetargetTargets(finger_flexion=finger_targets)


def filter_l20_command(
    command: list[int],
    *,
    last_command: list[int] | None,
    max_delta_per_cycle: int,
    lowpass_alpha: float,
) -> list[int]:
    current = [clamp_u8(value) for value in command]
    if last_command is None:
        return current

    rate_limited: list[int] = []
    for previous, value in zip(last_command, current):
        delta = max(-max_delta_per_cycle, min(max_delta_per_cycle, value - previous))
        rate_limited.append(previous + delta)

    alpha = max(0.0, min(1.0, lowpass_alpha))
    return [
        clamp_u8(previous + alpha * (value - previous))
        for previous, value in zip(last_command, rate_limited)
    ]


def _joint_flexion_rad(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    first = a - b
    second = c - b
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= 1e-8:
        return 0.0
    cosine = float(np.dot(first, second) / denominator)
    interior_angle = math.acos(max(-1.0, min(1.0, cosine)))
    return math.pi - interior_angle


def _directed_finger_flexion_rad(landmarks: np.ndarray, finger_index: int, *, root: bool) -> float:
    mcp, pip, dip, tip = FINGER_LANDMARKS[finger_index]
    if root:
        a, b, c = landmarks[mcp], landmarks[pip], landmarks[dip]
    else:
        a, b, c = landmarks[pip], landmarks[dip], landmarks[tip]
    frame = _palm_frame(landmarks)
    if frame is None:
        return _joint_flexion_rad(a, b, c)
    lateral, _forward, normal = frame
    reference_axis = normal if finger_index == 0 else lateral
    return _signed_joint_flexion_rad(a, b, c, reference_axis)


def _signed_joint_flexion_rad(
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    reference_axis: np.ndarray,
) -> float:
    unsigned = _joint_flexion_rad(a, b, c)
    first = np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)
    second = np.asarray(c, dtype=np.float64) - np.asarray(b, dtype=np.float64)
    bend_axis = np.cross(first, second)
    axis_norm = float(np.linalg.norm(bend_axis))
    ref_norm = float(np.linalg.norm(reference_axis))
    if axis_norm <= 1e-8 or ref_norm <= 1e-8:
        return unsigned
    sign = 1.0 if float(np.dot(bend_axis / axis_norm, reference_axis / ref_norm)) >= 0.0 else -1.0
    return sign * unsigned


def _normalized_angle(
    angle: float,
    open_angle: float,
    closed_angle: float,
    gamma: float,
    *,
    open_deadband_rad: float = 0.0,
) -> float:
    open_value = float(open_angle)
    if float(angle) <= open_value + max(0.0, float(open_deadband_rad)):
        return 0.0
    span = max(1e-6, float(closed_angle) - open_value)
    amount = max(0.0, min(1.0, (float(angle) - open_value) / span))
    return amount ** gamma


def _open_straightness_guard(
    amount: float,
    mcp: np.ndarray,
    pip: np.ndarray,
    dip: np.ndarray,
    tip: np.ndarray,
    threshold: float,
) -> float:
    if threshold <= 0.0:
        return float(amount)
    if _finger_straightness(mcp, pip, dip, tip) >= float(threshold):
        return 0.0
    return float(amount)


def _finger_straightness(mcp: np.ndarray, pip: np.ndarray, dip: np.ndarray, tip: np.ndarray) -> float:
    points = [
        np.asarray(mcp, dtype=np.float64),
        np.asarray(pip, dtype=np.float64),
        np.asarray(dip, dtype=np.float64),
        np.asarray(tip, dtype=np.float64),
    ]
    chain_length = sum(float(np.linalg.norm(end - start)) for start, end in zip(points[:-1], points[1:]))
    if chain_length <= 1e-8:
        return 0.0
    chord = float(np.linalg.norm(points[-1] - points[0]))
    return max(0.0, min(1.0, chord / chain_length))


def _lerp_command(open_value: int, closed_value: int, amount: float) -> int:
    return clamp_u8(float(open_value) + float(amount) * (float(closed_value) - float(open_value)))


def _scale_command_delta(value: int, neutral_value: int, scale: float) -> int:
    return clamp_u8(float(neutral_value) + float(scale) * (float(value) - float(neutral_value)))
