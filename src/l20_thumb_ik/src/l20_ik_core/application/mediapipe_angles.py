"""MediaPipe hand-landmark angle diagnostics."""

from __future__ import annotations

import math

import numpy as np

_FINGERS = (
    ("食指", 5, 6, 7, 8),
    ("中指", 9, 10, 11, 12),
    ("无名指", 13, 14, 15, 16),
    ("小拇指", 17, 18, 19, 20),
)


def format_mediapipe_flexion_line(landmarks_3d: np.ndarray) -> str:
    middle_angles, tip_angles = compute_mediapipe_flexion_rad(landmarks_3d)
    return _format_angles("mediapipe 3D原始几何弯曲角(弧度)", middle_angles, tip_angles)


def compute_mediapipe_flexion_rad(landmarks_3d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    landmarks = np.asarray(landmarks_3d, dtype=np.float64)
    if landmarks.shape != (21, 3):
        raise ValueError(f"Expected MediaPipe landmarks of shape (21, 3), got {landmarks.shape}")
    middle_angles: list[float] = []
    tip_angles: list[float] = []
    for _, base_index, middle_index, distal_index, tip_index in _FINGERS:
        middle_angles.append(
            _joint_flexion_rad(
                landmarks[base_index],
                landmarks[middle_index],
                landmarks[distal_index],
            )
        )
        tip_angles.append(
            _joint_flexion_rad(
                landmarks[middle_index],
                landmarks[distal_index],
                landmarks[tip_index],
            )
        )
    return np.asarray(middle_angles, dtype=np.float64), np.asarray(tip_angles, dtype=np.float64)


def _format_angles(title: str, middle_angles: np.ndarray, tip_angles: np.ndarray) -> str:
    return (
        f"{title}\n"
        f"  手指={[name for name, *_ in _FINGERS]}\n"
        f"  指中弯曲={[round(float(value), 4) for value in middle_angles]}\n"
        f"  指尖弯曲={[round(float(value), 4) for value in tip_angles]}"
    )


def _joint_flexion_rad(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    first = a - b
    second = c - b
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= 0.0:
        raise ValueError("Cannot compute MediaPipe flexion angle from coincident landmarks")
    cosine = float(np.dot(first, second) / denominator)
    interior_angle = math.acos(max(-1.0, min(1.0, cosine)))
    return math.pi - interior_angle
