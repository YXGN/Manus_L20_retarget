"""Post-retarget L20 flexion command calibration."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from l20_ik_core.infrastructure.hand_model import HandModel

_SIDE_PREFIXES = ("lh", "rh", "left", "right", "l", "r")
_JOINT_ALIASES = {
    "thumb_dip": ("thumb_ip",),
    "thumb_ip": ("thumb_dip",),
}
_FLEXION_COMMAND_GROUPS = {
    "指根弯曲": (
        [0, 1, 2, 3, 4],
        [
            "thumb_cmc_pitch",
            "index_mcp_pitch",
            "middle_mcp_pitch",
            "ring_mcp_pitch",
            "pinky_mcp_pitch",
        ],
    ),
    "指尖弯曲": (
        [15, 16, 17, 18, 19],
        [
            "thumb_dip",
            "index_dip",
            "middle_dip",
            "ring_dip",
            "pinky_dip",
        ],
    ),
}


class L20FlexionCommandCalibrator:
    """Maps calibrated retarget flexion qpos to L20 0-255 command slots."""

    def __init__(
        self,
        hand_model: HandModel,
        calibration_path: str,
        *,
        deadzone: float = 0.0,
        base_follow_tip: float = 0.0,
        tip_gamma: float = 1.0,
        close_command_floor: int = 0,
    ):
        self._joint_index = hand_model.get_joint_name_to_qpos_index()
        self._deadzone = float(deadzone)
        self._base_follow_tip = float(base_follow_tip)
        self._tip_gamma = float(tip_gamma)
        self._close_command_floor = int(close_command_floor)
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError(f"L20 flexion calibration deadzone must be in [0, 1), got {deadzone}")
        if not 0.0 <= self._base_follow_tip <= 1.0:
            raise ValueError(
                f"L20 flexion base-follow-tip must be in [0, 1], got {base_follow_tip}"
            )
        if self._tip_gamma <= 0.0:
            raise ValueError(f"L20 flexion tip gamma must be > 0, got {tip_gamma}")
        if not 0 <= self._close_command_floor <= 255:
            raise ValueError(
                f"L20 flexion close command floor must be in [0, 255], got {close_command_floor}"
            )
        self._groups = self._load_groups(calibration_path)

    def apply(self, qpos: np.ndarray, command: list[int]) -> list[int]:
        values = np.asarray(qpos, dtype=np.float64)
        if values.ndim != 1:
            raise ValueError(f"Expected qpos to be one-dimensional, got shape {values.shape}")
        if len(command) != 20:
            raise ValueError(f"L20 flexion calibration requires 20 command values, got {len(command)}")

        calibrated = list(command)
        for group_name, group in self._groups.items():
            slots, joint_names = _FLEXION_COMMAND_GROUPS[group_name]
            open_values = group["open"]
            close_values = group["close"]
            for slot, joint_name, open_value, close_value in zip(
                slots,
                joint_names,
                open_values,
                close_values,
                strict=True,
            ):
                denominator = close_value - open_value
                if abs(denominator) < 1e-8:
                    raise ValueError(
                        f"L20 flexion calibration has zero range for {group_name}/{joint_name}"
                    )
                qpos_value = float(values[self._resolve_joint_index(joint_name)])
                normalized = (qpos_value - open_value) / denominator
                normalized = float(np.clip(normalized, 0.0, 1.0))
                normalized = self._apply_deadzone(normalized)
                if group_name == "指尖弯曲":
                    normalized = normalized ** self._tip_gamma
                calibrated[slot] = self._normalized_to_command(normalized)
        if self._base_follow_tip > 0.0:
            self._apply_base_follow_tip(calibrated)
        self._apply_low_angle_root_offsets(values, calibrated)
        return calibrated

    def _apply_deadzone(self, value: float) -> float:
        if self._deadzone <= 0.0:
            return value
        if value <= self._deadzone:
            return 0.0
        return (value - self._deadzone) / (1.0 - self._deadzone)

    def _normalized_to_command(self, value: float) -> int:
        close_range = 255 - self._close_command_floor
        return int(round(255.0 - close_range * value))

    def _apply_base_follow_tip(self, command: list[int]) -> None:
        base_slots, _ = _FLEXION_COMMAND_GROUPS["指根弯曲"]
        tip_slots, _ = _FLEXION_COMMAND_GROUPS["指尖弯曲"]
        for base_slot, tip_slot in zip(base_slots, tip_slots, strict=True):
            tip_value = self._command_to_normalized(command[tip_slot])
            base_value = self._command_to_normalized(command[base_slot])
            followed_base = max(base_value, tip_value * self._base_follow_tip)
            command[base_slot] = self._normalized_to_command(followed_base)

    def _apply_low_angle_root_offsets(self, qpos: np.ndarray, command: list[int]) -> None:
        group = self._groups.get("指根弯曲")
        if group is None:
            return
        offsets = group.get("low_angle_normalized_offset")
        knees = group.get("low_angle_knee")
        if offsets is None or knees is None:
            return
        slots, joint_names = _FLEXION_COMMAND_GROUPS["指根弯曲"]
        open_values = group["open"]
        for slot, joint_name, open_value, offset, knee in zip(
            slots,
            joint_names,
            open_values,
            offsets,
            knees,
            strict=True,
        ):
            if offset <= 0.0 or knee <= 0.0:
                continue
            qpos_value = float(qpos[self._resolve_joint_index(joint_name)])
            angle_from_open = max(qpos_value - open_value, 0.0)
            fade = float(np.clip(1.0 - angle_from_open / knee, 0.0, 1.0))
            normalized = self._command_to_normalized(command[slot])
            corrected = float(np.clip(normalized - offset * fade, 0.0, 1.0))
            command[slot] = self._normalized_to_command(corrected)

    def _command_to_normalized(self, command_value: int) -> float:
        close_range = 255 - self._close_command_floor
        if close_range <= 0:
            return 0.0
        return float(np.clip((255.0 - float(command_value)) / float(close_range), 0.0, 1.0))

    def _resolve_joint_index(self, joint_name: str) -> int:
        if joint_name in self._joint_index:
            return self._joint_index[joint_name]
        for alias in _JOINT_ALIASES.get(joint_name, ()):
            if alias in self._joint_index:
                return self._joint_index[alias]
        for prefix in _SIDE_PREFIXES:
            candidate = f"{prefix}_{joint_name}"
            if candidate in self._joint_index:
                return self._joint_index[candidate]
            for alias in _JOINT_ALIASES.get(joint_name, ()):
                candidate = f"{prefix}_{alias}"
                if candidate in self._joint_index:
                    return self._joint_index[candidate]
        raise KeyError(f"Cannot find joint '{joint_name}' in retarget hand model")

    @staticmethod
    def _load_groups(calibration_path: str) -> dict[str, dict[str, list[float]]]:
        path = Path(calibration_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"L20 flexion calibration file not found: {path}")
        payload = json.loads(path.read_text(encoding="utf-8"))
        schema = payload.get("schema")
        if schema != "l20_ik_core.l20_flexion_linear_calibration.v1":
            raise ValueError(f"Unsupported L20 flexion calibration schema: {schema!r}")
        raw_map = payload.get("linear_map")
        if not isinstance(raw_map, dict):
            raise ValueError("L20 flexion calibration JSON must contain object field 'linear_map'")

        groups: dict[str, dict[str, list[float]]] = {}
        for group_name, (_, expected_joints) in _FLEXION_COMMAND_GROUPS.items():
            raw_group = raw_map.get(group_name)
            if not isinstance(raw_group, dict):
                raise ValueError(f"L20 flexion calibration missing group: {group_name}")
            joints = raw_group.get("joints")
            if joints != expected_joints:
                raise ValueError(
                    f"L20 flexion calibration group {group_name} joints {joints!r} "
                    f"do not match expected {expected_joints!r}"
                )
            open_values = raw_group.get("open")
            close_values = raw_group.get("close")
            if not isinstance(open_values, list) or not isinstance(close_values, list):
                raise ValueError(f"L20 flexion calibration group {group_name} must contain open/close lists")
            if len(open_values) != 5 or len(close_values) != 5:
                raise ValueError(f"L20 flexion calibration group {group_name} must have 5 values")
            groups[group_name] = {
                "open": [float(value) for value in open_values],
                "close": [float(value) for value in close_values],
            }
            low_angle_offsets = raw_group.get("low_angle_normalized_offset")
            low_angle_knees = raw_group.get("low_angle_knee")
            if low_angle_offsets is not None or low_angle_knees is not None:
                if not isinstance(low_angle_offsets, list) or not isinstance(low_angle_knees, list):
                    raise ValueError(
                        f"L20 flexion calibration group {group_name} low-angle fields must be lists"
                    )
                if len(low_angle_offsets) != 5 or len(low_angle_knees) != 5:
                    raise ValueError(
                        f"L20 flexion calibration group {group_name} low-angle fields must have 5 values"
                    )
                groups[group_name]["low_angle_normalized_offset"] = [
                    float(value) for value in low_angle_offsets
                ]
                groups[group_name]["low_angle_knee"] = [float(value) for value in low_angle_knees]
        return groups
