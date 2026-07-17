from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite, pi
from pathlib import Path
from typing import Any, Mapping

import yaml


COMMAND_LENGTH = 20
FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).expanduser().open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"YAML root must be an object: {path}")
    return payload


def deg_to_rad(value: float) -> float:
    return value * pi / 180.0


def rad_to_deg(value: float) -> float:
    return value * 180.0 / pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_u8(value: float | int) -> int:
    return int(round(clamp(float(value), 0.0, 255.0)))


def finite_or(value: object, fallback: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    return number if isfinite(number) else fallback


def ergonomics_value(ergonomics: Mapping[str, float], name: str, fallback: float = 0.0) -> float:
    return finite_or(ergonomics.get(name), fallback)


def _command_list(value: object, fallback: int) -> list[int]:
    if isinstance(value, list) and len(value) == COMMAND_LENGTH:
        return [clamp_u8(item) for item in value]
    return [clamp_u8(fallback)] * COMMAND_LENGTH


@dataclass(slots=True)
class RevoStyleL20Retarget:
    config: Mapping[str, Any]
    _last_command: list[int] | None = None
    _last_target: list[int] | None = None
    _neutral: list[int] = field(init=False)
    _min_command: list[int] = field(init=False)
    _max_command: list[int] = field(init=False)
    _open_command: list[int] = field(init=False)
    _closed_command: list[int] = field(init=False)
    _alpha: float = field(init=False)
    _max_delta: int = field(init=False)

    def __post_init__(self) -> None:
        command_cfg = self.config.get("l20_command", {})
        self._neutral = _command_list(command_cfg.get("neutral_command"), 128)
        self._min_command = _command_list(command_cfg.get("min_command"), 0)
        self._max_command = _command_list(command_cfg.get("max_command"), 255)
        self._open_command = _command_list(command_cfg.get("open_command"), 255)
        self._closed_command = _command_list(command_cfg.get("closed_command"), 0)
        smoothing = self.config.get("smoothing", {})
        self._alpha = clamp(finite_or(smoothing.get("lowpass_alpha"), 1.0), 0.0, 1.0)
        self._max_delta = max(0, int(finite_or(smoothing.get("max_delta_per_cycle"), 0)))

    def retarget(self, ergonomics: Mapping[str, float], *, smooth: bool = True) -> tuple[dict[str, float], list[int]]:
        q = self.compute_joint_targets(ergonomics)
        command = self.joints_to_l20_command(q, ergonomics)
        self._last_target = command
        if smooth:
            command = self._smooth(command)
        self._last_command = command
        return q, command

    def compute_joint_targets(self, ergonomics: Mapping[str, float]) -> dict[str, float]:
        input_cfg = self.config.get("input", {})
        neutral = input_cfg.get("neutral_ergonomics", {}) if isinstance(input_cfg, Mapping) else {}
        four = self.config.get("four_finger", {})
        four = four if isinstance(four, Mapping) else {}
        spread = self.config.get("spread", {})
        spread = spread if isinstance(spread, Mapping) else {}
        thumb = self.config.get("thumb", {})
        thumb = thumb if isinstance(thumb, Mapping) else {}

        def flex(name: str) -> float:
            return ergonomics_value(ergonomics, name, 0.0) - finite_or(neutral.get(name), 0.0)

        q: dict[str, float] = {}
        index_scale = finite_or(four.get("index_angle_scale"), 1.0)
        all_scale = finite_or(four.get("all_finger_angle_scale"), 1.0)
        mcp_scale = finite_or(four.get("four_finger_mcp_scale"), 1.0)
        middle_ring_dip_scale = finite_or(four.get("middle_ring_dip_scale"), 1.0)
        pinky_scale = finite_or(four.get("pinky_angle_scale"), 1.0)
        pinky_mcp_scale = finite_or(four.get("pinky_mcp_scale"), 1.0)
        pinky_dip_pip_scale = finite_or(four.get("pinky_dip_pip_scale"), 1.0)

        q["index_mcp"] = deg_to_rad(flex("IndexMCPStretch") * index_scale * mcp_scale)
        q["index_pip"] = deg_to_rad(flex("IndexPIPStretch") * index_scale)
        q["index_dip"] = deg_to_rad(flex("IndexDIPStretch") * index_scale)

        q["middle_mcp"] = deg_to_rad(flex("MiddleMCPStretch") * all_scale * mcp_scale)
        q["middle_pip"] = deg_to_rad(flex("MiddlePIPStretch") * all_scale)
        q["middle_dip"] = deg_to_rad(flex("MiddleDIPStretch") * all_scale * middle_ring_dip_scale)

        q["ring_mcp"] = deg_to_rad(flex("RingMCPStretch") * all_scale * mcp_scale)
        q["ring_pip"] = deg_to_rad(flex("RingPIPStretch") * all_scale)
        q["ring_dip"] = deg_to_rad(flex("RingDIPStretch") * all_scale * middle_ring_dip_scale)

        q["pinky_mcp"] = deg_to_rad(flex("PinkyMCPStretch") * pinky_scale * pinky_mcp_scale * mcp_scale)
        q["pinky_pip"] = deg_to_rad(flex("PinkyPIPStretch") * pinky_scale * pinky_dip_pip_scale)
        q["pinky_dip"] = deg_to_rad(flex("PinkyDIPStretch") * pinky_scale * pinky_dip_pip_scale)

        middle_ref = ergonomics_value(ergonomics, "MiddleSpread", 0.0)
        sign = finite_or(spread.get("finger_spread_sign"), -1.0)
        middle_dynamic = bool(spread.get("middle_dynamic", True))
        middle_value_deg = (
            (middle_ref - finite_or(spread.get("middle_offset_deg"), 0.0)) * finite_or(spread.get("middle_scale"), 1.0)
            if middle_dynamic
            else -finite_or(spread.get("middle_offset_deg"), 0.0)
        )
        index_value_deg = (
            ergonomics_value(ergonomics, "IndexSpread", 0.0)
            - middle_ref
            - finite_or(spread.get("index_offset_deg"), 0.0)
        ) * finite_or(spread.get("index_scale"), 1.0)
        ring_value_deg = (
            ergonomics_value(ergonomics, "RingSpread", 0.0)
            - middle_ref
            - finite_or(spread.get("ring_offset_deg"), 0.0)
        ) * finite_or(spread.get("ring_scale"), 1.0)
        pinky_value_deg = (
            ergonomics_value(ergonomics, "PinkySpread", 0.0)
            - middle_ref
            - finite_or(spread.get("pinky_offset_deg"), 0.0)
        ) * finite_or(spread.get("pinky_scale"), 1.0)
        ring_value_deg *= finite_or(
            spread.get("ring_forward_scale" if ring_value_deg > 0.0 else "ring_backward_scale"),
            1.0,
        )

        q["index_spread"] = deg_to_rad(sign * index_value_deg)
        q["middle_spread"] = deg_to_rad(sign * middle_value_deg)
        q["ring_spread"] = deg_to_rad(sign * ring_value_deg)
        q["pinky_spread"] = deg_to_rad(sign * pinky_value_deg)

        q["thumb_mcp"] = deg_to_rad(flex("ThumbMCPStretch") * finite_or(thumb.get("mcp_scale"), 1.0))
        q["thumb_pip"] = deg_to_rad(flex("ThumbPIPStretch") * finite_or(thumb.get("pip_scale"), 1.0))
        q["thumb_dip"] = deg_to_rad(flex("ThumbDIPStretch") * finite_or(thumb.get("dip_scale"), 1.0))
        spread_key = str(thumb.get("spread_key", "ThumbMCPSpread"))
        thumb_spread_deg = (
            ergonomics_value(ergonomics, spread_key, 0.0) - finite_or(thumb.get("spread_offset_deg"), 0.0)
        ) * finite_or(thumb.get("spread_scale"), -1.0)
        q["thumb_roll"] = deg_to_rad(thumb_spread_deg + finite_or(thumb.get("mcp_offset_deg"), 0.0))
        q["thumb_yaw"] = deg_to_rad(thumb_spread_deg)

        return self._apply_joint_calibration(q)

    def _apply_joint_calibration(self, q: dict[str, float]) -> dict[str, float]:
        calibration = self.config.get("joint_calibration", {})
        default_scale = finite_or(calibration.get("default_scale"), 1.0)
        default_offset = deg_to_rad(finite_or(calibration.get("default_offset_deg"), 0.0))
        joints = calibration.get("joints", {})
        calibrated: dict[str, float] = {}
        for name, value in q.items():
            raw = joints.get(name, {}) if isinstance(joints, Mapping) else {}
            scale = finite_or(raw.get("scale"), default_scale) if isinstance(raw, Mapping) else default_scale
            offset = deg_to_rad(finite_or(raw.get("offset_deg"), 0.0)) if isinstance(raw, Mapping) else default_offset
            calibrated[name] = value * scale + offset
        return calibrated

    def joints_to_l20_command(
        self,
        q: Mapping[str, float],
        ergonomics: Mapping[str, float] | None = None,
    ) -> list[int]:
        command = list(self._neutral)
        direct = bool(self.config.get("l20_command", {}).get("direct_ergonomics_mapping", False))
        if direct and ergonomics is not None:
            self._set_direct_flexion_slots(command, ergonomics)
        else:
            self._set_joint_slot(command, 0, "thumb_mcp", q)
            self._set_joint_slot(command, 1, "index_mcp", q)
            self._set_joint_slot(command, 2, "middle_mcp", q)
            self._set_joint_slot(command, 3, "ring_mcp", q)
            self._set_joint_slot(command, 4, "pinky_mcp", q)

        self._set_joint_slot(command, 5, "thumb_roll", q)
        self._set_joint_slot(command, 6, "index_spread", q)
        self._set_joint_slot(command, 7, "middle_spread", q)
        self._set_joint_slot(command, 8, "ring_spread", q)
        self._set_joint_slot(command, 9, "pinky_spread", q)
        self._set_joint_slot(command, 10, "thumb_yaw", q)

        if not (direct and ergonomics is not None):
            self._set_tip_slot(command, 15, "thumb", q)
            self._set_tip_slot(command, 16, "index", q)
            self._set_tip_slot(command, 17, "middle", q)
            self._set_tip_slot(command, 18, "ring", q)
            self._set_tip_slot(command, 19, "pinky", q)

        for index, value in enumerate(command):
            command[index] = clamp_u8(clamp(value, self._min_command[index], self._max_command[index]))
        return command

    def _set_direct_flexion_slots(self, command: list[int], ergonomics: Mapping[str, float]) -> None:
        slot_map = {
            "thumb": (0, 15),
            "index": (1, 16),
            "middle": (2, 17),
            "ring": (3, 18),
            "pinky": (4, 19),
        }
        range_cfg = self.config.get("l20_command", {}).get("ergonomics_ranges", {})
        if not isinstance(range_cfg, Mapping):
            return
        for finger, (root_slot, tip_slot) in slot_map.items():
            finger_cfg = range_cfg.get(finger, {})
            if not isinstance(finger_cfg, Mapping):
                continue
            root_amount = self._ergonomics_amount(ergonomics, finger_cfg.get("root_key"), finger_cfg.get("root_range"))
            tip_amount = self._ergonomics_amount(ergonomics, finger_cfg.get("tip_key"), finger_cfg.get("tip_range"))
            if root_amount is not None:
                command[root_slot] = clamp_u8(
                    self._open_command[root_slot]
                    + root_amount * (self._closed_command[root_slot] - self._open_command[root_slot])
                )
            if tip_amount is not None:
                command[tip_slot] = clamp_u8(
                    self._open_command[tip_slot]
                    + tip_amount * (self._closed_command[tip_slot] - self._open_command[tip_slot])
                )

    def _ergonomics_amount(
        self,
        ergonomics: Mapping[str, float],
        key_value: object,
        range_value: object,
    ) -> float | None:
        if key_value is None:
            return None
        key = str(key_value)
        if key not in ergonomics:
            return None
        if not isinstance(range_value, list) or len(range_value) < 2:
            return None
        low = finite_or(range_value[0], 0.0)
        high = finite_or(range_value[1], 1.0)
        if abs(high - low) < 1e-9:
            return 0.0
        return clamp((ergonomics_value(ergonomics, key, low) - low) / (high - low), 0.0, 1.0)

    def _set_joint_slot(self, command: list[int], slot: int, joint_name: str, q: Mapping[str, float]) -> None:
        value_deg = rad_to_deg(finite_or(q.get(joint_name), 0.0))
        amount = self._normalized_amount(joint_name, value_deg)
        command[slot] = clamp_u8(self._open_command[slot] + amount * (self._closed_command[slot] - self._open_command[slot]))

    def _set_tip_slot(self, command: list[int], slot: int, finger: str, q: Mapping[str, float]) -> None:
        if finger == "thumb":
            value_deg = 0.45 * rad_to_deg(finite_or(q.get("thumb_pip"), 0.0)) + 0.55 * rad_to_deg(
                finite_or(q.get("thumb_dip"), 0.0)
            )
            range_name = "thumb_tip"
        else:
            value_deg = 0.45 * rad_to_deg(finite_or(q.get(f"{finger}_pip"), 0.0)) + 0.55 * rad_to_deg(
                finite_or(q.get(f"{finger}_dip"), 0.0)
            )
            range_name = f"{finger}_tip"
        amount = self._normalized_amount(range_name, value_deg)
        command[slot] = clamp_u8(self._open_command[slot] + amount * (self._closed_command[slot] - self._open_command[slot]))

    def _normalized_amount(self, range_name: str, value_deg: float) -> float:
        ranges = self.config.get("l20_command", {}).get("joint_ranges_deg", {})
        raw = ranges.get(range_name, [0.0, 1.0]) if isinstance(ranges, Mapping) else [0.0, 1.0]
        low = finite_or(raw[0], 0.0) if isinstance(raw, list) and len(raw) >= 2 else 0.0
        high = finite_or(raw[1], 1.0) if isinstance(raw, list) and len(raw) >= 2 else 1.0
        if abs(high - low) < 1e-9:
            return 0.0
        return clamp((value_deg - low) / (high - low), 0.0, 1.0)

    def _smooth(self, command: list[int]) -> list[int]:
        if self._last_command is None:
            return [clamp_u8(value) for value in command]
        out: list[int] = []
        for previous, target in zip(self._last_command, command):
            limited_target = target
            if self._max_delta > 0:
                limited_target = int(previous + clamp(target - previous, -self._max_delta, self._max_delta))
            out.append(clamp_u8(previous + self._alpha * (limited_target - previous)))
        return out
