"""Post-retarget L20 joint range command mapping."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from somehand.infrastructure.hand_model import HandModel

_SIDE_PREFIXES = ("lh", "rh", "left", "right", "l", "r")
_JOINT_ALIASES = {
    "thumb_dip": ("thumb_ip",),
    "thumb_ip": ("thumb_dip",),
}


class L20JointRangeCommandMapper:
    """Maps selected L20 qpos joints directly into 0-255 command slots."""

    def __init__(self, hand_model: HandModel, mapping_path: str):
        self._joint_index = hand_model.get_joint_name_to_qpos_index()
        self._entries = self._load_entries(mapping_path)
        if not self._entries:
            raise ValueError(f"L20 joint range mapping has no joints: {mapping_path}")

    def apply(self, qpos: np.ndarray, command: list[int]) -> list[int]:
        values = np.asarray(qpos, dtype=np.float64)
        if values.ndim != 1:
            raise ValueError(f"Expected qpos to be one-dimensional, got shape {values.shape}")
        if len(command) != 20:
            raise ValueError(f"L20 joint range mapping requires 20 command values, got {len(command)}")

        mapped = list(command)
        for entry in self._entries:
            qpos_value = float(values[self._resolve_joint_index(str(entry["joint"]))])
            sim_min = float(entry["sim_min"])
            sim_max = float(entry["sim_max"])
            denominator = sim_max - sim_min
            if abs(denominator) < 1e-8:
                raise ValueError(f"L20 joint range mapping has zero sim range for {entry['joint']}")
            normalized = float(np.clip((qpos_value - sim_min) / denominator, 0.0, 1.0))
            gamma = float(entry["gamma"])
            normalized = normalized ** gamma
            normalized = self._apply_post_compensation(normalized, entry)
            command_min = float(entry["command_min"])
            command_max = float(entry["command_max"])
            command_value = command_min + normalized * (command_max - command_min)
            mapped[int(entry["command_slot"])] = int(round(np.clip(command_value, 0.0, 255.0)))
        return mapped

    @staticmethod
    def _apply_post_compensation(normalized: float, entry: dict[str, float | int | str]) -> float:
        threshold = entry.get("post_threshold")
        gain = entry.get("post_gain")
        if threshold is None or gain is None:
            return normalized
        threshold_value = float(threshold)
        gain_value = float(gain)
        if normalized <= threshold_value:
            return normalized
        tail_range = max(1.0 - threshold_value, 1e-8)
        tail_normalized = float(np.clip((normalized - threshold_value) / tail_range, 0.0, 1.0))
        power = float(entry.get("post_power", 1.0))
        compensated = normalized + gain_value * (tail_normalized ** power)
        return float(np.clip(compensated, 0.0, 1.0))

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
    def _load_entries(mapping_path: str) -> list[dict[str, float | int | str]]:
        path = Path(mapping_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"L20 joint range mapping file not found: {path}")
        payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        raw_joints = payload.get("joints")
        if not isinstance(raw_joints, dict):
            raise ValueError("L20 joint range mapping YAML must contain object field 'joints'")

        entries: list[dict[str, float | int | str]] = []
        for joint_name, raw_entry in raw_joints.items():
            if not isinstance(raw_entry, dict):
                raise ValueError(f"L20 joint range mapping for {joint_name} must be an object")
            required_keys = ("command_slot", "sim_min", "sim_max", "command_min", "command_max")
            missing = [key for key in required_keys if key not in raw_entry]
            if missing:
                raise ValueError(f"L20 joint range mapping for {joint_name} is missing keys: {missing}")
            command_slot = int(raw_entry["command_slot"])
            if not 0 <= command_slot < 20:
                raise ValueError(
                    f"L20 joint range mapping command_slot for {joint_name} must be in [0, 19], got {command_slot}"
                )
            command_min = float(raw_entry["command_min"])
            command_max = float(raw_entry["command_max"])
            gamma = float(raw_entry.get("gamma", 1.0))
            post_threshold = raw_entry.get("post_threshold")
            post_gain = raw_entry.get("post_gain")
            post_power = raw_entry.get("post_power", 1.0)
            if not 0.0 <= command_min <= 255.0:
                raise ValueError(
                    f"L20 joint range mapping command_min for {joint_name} must be in [0, 255], got {command_min}"
                )
            if not 0.0 <= command_max <= 255.0:
                raise ValueError(
                    f"L20 joint range mapping command_max for {joint_name} must be in [0, 255], got {command_max}"
                )
            if gamma <= 0.0:
                raise ValueError(
                    f"L20 joint range mapping gamma for {joint_name} must be > 0, got {gamma}"
                )
            if post_threshold is not None:
                post_threshold = float(post_threshold)
                if not 0.0 <= post_threshold < 1.0:
                    raise ValueError(
                        f"L20 joint range mapping post_threshold for {joint_name} must be in [0, 1), got {post_threshold}"
                    )
            if post_gain is not None:
                post_gain = float(post_gain)
                if post_gain < 0.0:
                    raise ValueError(
                        f"L20 joint range mapping post_gain for {joint_name} must be >= 0, got {post_gain}"
                    )
            post_power = float(post_power)
            if post_power <= 0.0:
                raise ValueError(
                    f"L20 joint range mapping post_power for {joint_name} must be > 0, got {post_power}"
                )
            if (post_threshold is None) != (post_gain is None):
                raise ValueError(
                    f"L20 joint range mapping for {joint_name} must set post_threshold and post_gain together"
                )
            entries.append(
                {
                    "joint": str(joint_name),
                    "command_slot": command_slot,
                    "sim_min": float(raw_entry["sim_min"]),
                    "sim_max": float(raw_entry["sim_max"]),
                    "command_min": command_min,
                    "command_max": command_max,
                    "gamma": gamma,
                    **({} if post_threshold is None else {"post_threshold": post_threshold}),
                    **({} if post_gain is None else {"post_gain": post_gain}),
                    "post_power": post_power,
                }
            )
        return entries
