from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

FINGER_ORDER = ("thumb", "index", "middle", "ring", "pinky")
COMMAND_LENGTH = 20


def clamp_u8(value: float | int) -> int:
    return max(0, min(255, int(round(float(value)))))


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


@dataclass(slots=True)
class ManusToL20Mapper:
    config: dict[str, Any]

    def map_ergonomics(self, ergonomics: dict[str, float]) -> list[int]:
        command_cfg = self.config.get("command", {})
        fingers = self.config.get("fingers", {})
        open_value = int(command_cfg.get("open_value", 0))
        closed_value = int(command_cfg.get("closed_value", 255))
        reserved_value = int(command_cfg.get("reserved_value", 0))
        tip_follow_root = float(command_cfg.get("tip_follow_root", 0.85))
        open_command = _command_list(command_cfg.get("open_command"), open_value)
        closed_command = _command_list(command_cfg.get("closed_command"), closed_value)
        default_command = _command_list(command_cfg.get("default_command"), reserved_value)
        gamma_command = _float_command_list(command_cfg.get("gamma_command"), 1.0)
        post_threshold = _optional_float_command_list(command_cfg.get("post_threshold_command"))
        post_gain = _optional_float_command_list(command_cfg.get("post_gain_command"))
        post_power = _float_command_list(command_cfg.get("post_power_command"), 1.0)

        values = list(default_command)
        amounts: dict[str, dict[str, float]] = {}
        for index, finger in enumerate(FINGER_ORDER):
            finger_cfg = fingers.get(finger, {})
            root = _normalized(ergonomics, finger_cfg.get("root"), finger_cfg.get("root_range"))
            tip_key = finger_cfg.get("tip")
            tip = _normalized(ergonomics, tip_key, finger_cfg.get("tip_range"))
            spread = _normalized(
                ergonomics,
                finger_cfg.get("spread"),
                finger_cfg.get("spread_range"),
                default=0.5,
            )
            tip_amount = tip if tip_key in ergonomics else root * tip_follow_root
            amounts[finger] = {
                "root": root,
                "tip": tip_amount,
                "spread": spread,
            }

            values[index] = _lerp_slot(
                open_command,
                closed_command,
                gamma_command,
                post_threshold,
                post_gain,
                post_power,
                index,
                root,
            )
            values[5 + index] = _lerp_slot(
                open_command,
                closed_command,
                gamma_command,
                post_threshold,
                post_gain,
                post_power,
                5 + index,
                spread,
            )
            values[15 + index] = _lerp_slot(
                open_command,
                closed_command,
                gamma_command,
                post_threshold,
                post_gain,
                post_power,
                15 + index,
                tip_amount,
            )

        thumb_yaw_source = command_cfg.get("thumb_yaw_source")
        if thumb_yaw_source in ("root", "tip", "spread"):
            values[10] = _lerp_slot(
                open_command,
                closed_command,
                gamma_command,
                post_threshold,
                post_gain,
                post_power,
                10,
                amounts["thumb"][thumb_yaw_source],
            )
        elif command_cfg.get("thumb_yaw_from_thumb_spread", True):
            thumb_yaw_source_slot = int(command_cfg.get("thumb_yaw_source_slot", 5))
            if 0 <= thumb_yaw_source_slot < COMMAND_LENGTH:
                values[10] = values[thumb_yaw_source_slot]
        return values


class SafetyFilter:
    def __init__(self, config: dict[str, Any]):
        safety = config.get("safety", {})
        self.min_command = _command_list(safety.get("min_command"), 0)
        self.max_command = _command_list(safety.get("max_command"), 255)
        self.max_delta_per_cycle = int(safety.get("max_delta_per_cycle", 8))
        self.lowpass_alpha = float(safety.get("lowpass_alpha", 0.35))
        self.estop_command = _command_list(safety.get("estop_command"), 0)
        self._last: list[int] | None = None

    def apply(self, command: list[int], *, estop: bool = False) -> list[int]:
        if estop:
            safe = list(self.estop_command)
            self._last = safe
            return safe

        limited = [
            max(self.min_command[index], min(self.max_command[index], clamp_u8(value)))
            for index, value in enumerate(command)
        ]
        if self._last is None:
            self._last = limited
            return limited

        rate_limited: list[int] = []
        for previous, current in zip(self._last, limited):
            delta = max(-self.max_delta_per_cycle, min(self.max_delta_per_cycle, current - previous))
            rate_limited.append(previous + delta)

        alpha = max(0.0, min(1.0, self.lowpass_alpha))
        filtered = [
            clamp_u8(previous + alpha * (current - previous))
            for previous, current in zip(self._last, rate_limited)
        ]
        self._last = filtered
        return filtered


def _normalized(
    ergonomics: dict[str, float],
    key: str | None,
    value_range: list[float] | tuple[float, float] | None,
    *,
    default: float = 0.0,
) -> float:
    if not key or key not in ergonomics:
        return max(0.0, min(1.0, float(default)))
    low, high = (value_range or [0.0, 1.0])[:2]
    low = float(low)
    high = float(high)
    if high == low:
        return 0.0
    return max(0.0, min(1.0, (float(ergonomics[key]) - low) / (high - low)))


def _lerp_u8(open_value: int, closed_value: int, amount: float) -> int:
    return clamp_u8(open_value + amount * (closed_value - open_value))


def _lerp_slot(
    open_command: list[int],
    closed_command: list[int],
    gamma_command: list[float],
    post_threshold: list[float | None],
    post_gain: list[float | None],
    post_power: list[float],
    index: int,
    amount: float,
) -> int:
    gamma = max(0.001, float(gamma_command[index]))
    shaped = max(0.0, min(1.0, float(amount))) ** gamma
    shaped = _apply_post_compensation(shaped, post_threshold[index], post_gain[index], post_power[index])
    return _lerp_u8(open_command[index], closed_command[index], shaped)


def _apply_post_compensation(
    amount: float,
    threshold: float | None,
    gain: float | None,
    power: float,
) -> float:
    if threshold is None or gain is None:
        return amount
    threshold_value = float(threshold)
    if amount <= threshold_value:
        return amount
    tail_range = max(1.0 - threshold_value, 1e-8)
    tail_normalized = max(0.0, min(1.0, (amount - threshold_value) / tail_range))
    compensated = amount + float(gain) * (tail_normalized ** max(0.001, float(power)))
    return max(0.0, min(1.0, compensated))


def _command_list(value: object, default: int) -> list[int]:
    if isinstance(value, list) and len(value) == COMMAND_LENGTH:
        return [clamp_u8(item) for item in value]
    return [clamp_u8(default)] * COMMAND_LENGTH


def _float_command_list(value: object, default: float) -> list[float]:
    if isinstance(value, list) and len(value) == COMMAND_LENGTH:
        return [float(item) for item in value]
    return [float(default)] * COMMAND_LENGTH


def _optional_float_command_list(value: object) -> list[float | None]:
    if isinstance(value, list) and len(value) == COMMAND_LENGTH:
        return [None if item is None else float(item) for item in value]
    return [None] * COMMAND_LENGTH
