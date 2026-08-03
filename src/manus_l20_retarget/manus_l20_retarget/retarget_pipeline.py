from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from manus_ros2_msgs.msg import ManusGlove

from .manus_landmarks import (
    ManusHandSkeleton,
    manus_hand_skeleton_to_mediapipe_landmarks,
    manus_raw_nodes_to_hand_skeleton,
)
from .mapping import clamp_u8


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
    """MANUS frame data for the retarget pipeline."""

    skeleton: ManusHandSkeleton
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
    skeleton = manus_raw_nodes_to_hand_skeleton(
        msg.raw_nodes,
        transform=landmark_transform,
    )
    landmarks = manus_hand_skeleton_to_mediapipe_landmarks(skeleton)
    ergonomics = {entry.type: float(entry.value) for entry in msg.ergonomics if entry.type}
    return HandFeatures(
        skeleton=skeleton,
        landmarks=landmarks,
        ergonomics=ergonomics,
    )


def compute_ergonomics_flexion_targets(
    ergonomics: dict[str, float],
    *,
    root_keys: list[str],
    tip_key_groups: list[list[str]],
    root_open_values: list[float],
    root_closed_values: list[float],
    tip_open_values: list[float],
    tip_closed_values: list[float],
    root_gamma: float,
    tip_gamma: float,
) -> HandRetargetTargets:
    """Map calibrated MANUS ergonomics values to four-finger flexion targets."""
    if not (
        len(root_keys) == len(tip_key_groups) == len(root_open_values) == len(root_closed_values) == 4
        and len(tip_open_values) == len(tip_closed_values) == 4
    ):
        raise ValueError("four-finger ergonomics flexion calibration must contain four fingers")

    targets: dict[int, FingerFlexionTarget] = {}
    for local_index, root_key in enumerate(root_keys):
        root_value = float(ergonomics[root_key])
        tip_value = sum(float(ergonomics[key]) for key in tip_key_groups[local_index])
        targets[local_index + 1] = FingerFlexionTarget(
            root_amount=_normalized_calibration_value(
                root_value,
                root_open_values[local_index],
                root_closed_values[local_index],
                root_gamma,
            ),
            tip_amount=_normalized_calibration_value(
                tip_value,
                tip_open_values[local_index],
                tip_closed_values[local_index],
                tip_gamma,
            ),
        )
    return HandRetargetTargets(finger_flexion=targets)


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


def _normalized_calibration_value(
    value: float,
    open_value: float,
    closed_value: float,
    gamma: float,
) -> float:
    """Normalize a calibrated scalar while supporting either MANUS sign convention."""
    span = float(closed_value) - float(open_value)
    if abs(span) <= 1e-8:
        return 0.0
    amount = max(0.0, min(1.0, (float(value) - float(open_value)) / span))
    return amount ** max(0.05, float(gamma))


def _lerp_command(open_value: int, closed_value: int, amount: float) -> int:
    return clamp_u8(float(open_value) + float(amount) * (float(closed_value) - float(open_value)))


def _scale_command_delta(value: int, neutral_value: int, scale: float) -> int:
    return clamp_u8(float(neutral_value) + float(scale) * (float(value) - float(neutral_value)))
