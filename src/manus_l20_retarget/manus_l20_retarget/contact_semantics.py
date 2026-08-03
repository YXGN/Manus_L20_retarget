from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable, Mapping

import numpy as np

from .mapping import clamp_u8


FINGERTIP_CONTACT_FINGERS = ("index", "middle", "ring", "pinky")
FINGERTIP_CONTACT_SLOTS = {
    # Contact takeover is local to the thumb and the selected finger.  The
    # other fingers (including their yaw slots) continue through teleoperation.
    "index": (0, 1, 5, 10, 15, 16),
    "middle": (0, 2, 5, 10, 15, 17),
    "ring": (0, 3, 5, 10, 15, 18),
    "pinky": (0, 4, 5, 10, 15, 19),
}


@dataclass(frozen=True, slots=True)
class FingertipContactProfile:
    finger: str
    natural_open_distance_p05_ratio: float
    contact_distance_p95_ratio: float
    contact_command: tuple[int, ...]
    override_slots: tuple[int, ...]
    max_command_delta: int


@dataclass(frozen=True, slots=True)
class FingertipContactConfig:
    enabled: bool
    profiles: dict[str, FingertipContactProfile]
    min_hold_sec: float
    release_hold_sec: float
    candidate_gap_ratio: float
    takeover_start_progress: float
    full_takeover_progress: float
    firm_contact_enter_activation: float
    firm_contact_release_activation: float
    activation_rise_sec: float
    activation_release_sec: float


@dataclass(frozen=True, slots=True)
class FingertipContactDecision:
    pair: str | None
    activation: float
    ratios: dict[str, float]
    event: str | None


@dataclass(frozen=True, slots=True)
class FingertipContactPhaseConfig:
    close_orientation_completion: float = 0.25
    close_flexion_start: float = 0.40
    release_flexion_open_completion: float = 0.18
    release_orientation_gamma: float = 2.5
    direction_change_deadband: float = 0.04

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "close_orientation_completion",
            _unit_interval_parameter(self.close_orientation_completion, 0.25, allow_one=True),
        )
        object.__setattr__(
            self,
            "close_flexion_start",
            _unit_interval_parameter(self.close_flexion_start, 0.40, allow_one=False),
        )
        object.__setattr__(
            self,
            "release_flexion_open_completion",
            _unit_interval_parameter(self.release_flexion_open_completion, 0.18, allow_one=True),
        )
        object.__setattr__(
            self,
            "release_orientation_gamma",
            max(0.05, _finite_float(self.release_orientation_gamma, 2.5)),
        )
        object.__setattr__(
            self,
            "direction_change_deadband",
            _unit_interval_parameter(self.direction_change_deadband, 0.04, allow_one=True),
        )


def thumb_fingertip_distance_ratios(skeleton: Any) -> dict[str, float]:
    """Measure thumb-to-finger tip distances in units of palm width."""
    thumb_tip = _point(skeleton.thumb.tip)
    index_mcp = _point(skeleton.index.mcp)
    pinky_mcp = _point(skeleton.pinky.mcp)
    palm_width = float(np.linalg.norm(index_mcp - pinky_mcp))
    if not math.isfinite(palm_width) or palm_width <= 1e-8:
        raise ValueError("cannot measure fingertip contact: palm width is degenerate")

    ratios: dict[str, float] = {}
    for finger in FINGERTIP_CONTACT_FINGERS:
        tip = _point(getattr(skeleton, finger).tip)
        distance = float(np.linalg.norm(thumb_tip - tip))
        if not math.isfinite(distance):
            raise ValueError(f"cannot measure fingertip contact: {finger} distance is not finite")
        ratios[finger] = distance / palm_width
    return ratios


def parse_fingertip_contact_config(data: Any) -> FingertipContactConfig:
    if not isinstance(data, dict):
        raise ValueError("fingertip contact calibration must be a mapping")
    if str(data.get("schema", "")).strip() != "manus_l20.fingertip_contact_semantics.v2":
        raise ValueError("unsupported fingertip contact calibration schema")

    runtime = data.get("runtime")
    contacts = data.get("contacts")
    if not isinstance(runtime, dict) or not isinstance(contacts, dict):
        raise ValueError("fingertip contact calibration needs runtime and contacts mappings")

    profiles: dict[str, FingertipContactProfile] = {}
    for finger in FINGERTIP_CONTACT_FINGERS:
        entry = contacts.get(finger)
        if not isinstance(entry, dict):
            raise ValueError(f"fingertip contact calibration missing {finger} contact")
        human = entry.get("human")
        robot = entry.get("robot")
        if not isinstance(human, dict) or not isinstance(robot, dict):
            raise ValueError(f"fingertip contact {finger} needs human and robot mappings")

        natural_open_distance = _positive_float(
            human.get("natural_open_distance_p05_ratio"),
            f"{finger}.natural_open_distance_p05_ratio",
        )
        contact_distance = _positive_float(
            human.get("contact_distance_p95_ratio"),
            f"{finger}.contact_distance_p95_ratio",
        )
        if natural_open_distance <= contact_distance:
            raise ValueError(
                f"fingertip contact {finger} natural-open distance must exceed contact distance"
            )

        command = _command(robot.get("contact_command"), f"{finger}.contact_command")
        override_slots = _slots(robot.get("override_slots"), finger)
        profiles[finger] = FingertipContactProfile(
            finger=finger,
            natural_open_distance_p05_ratio=natural_open_distance,
            contact_distance_p95_ratio=contact_distance,
            contact_command=command,
            override_slots=override_slots,
            max_command_delta=_nonnegative_int(robot.get("max_command_delta", 0), f"{finger}.max_command_delta"),
        )

    return FingertipContactConfig(
        enabled=bool(runtime.get("enabled", False)),
        profiles=profiles,
        min_hold_sec=_nonnegative_float(runtime.get("min_hold_sec", 0.10), "runtime.min_hold_sec"),
        release_hold_sec=_nonnegative_float(runtime.get("release_hold_sec", 0.05), "runtime.release_hold_sec"),
        candidate_gap_ratio=_nonnegative_float(
            runtime.get("candidate_gap_ratio", 0.02),
            "runtime.candidate_gap_ratio",
        ),
        takeover_start_progress=_unit_interval_float(
            runtime.get("takeover_start_progress", 0.35),
            "runtime.takeover_start_progress",
        ),
        full_takeover_progress=_full_takeover_progress(
            runtime.get("full_takeover_progress", 0.75),
            runtime.get("takeover_start_progress", 0.35),
        ),
        firm_contact_enter_activation=_unit_interval_parameter(
            runtime.get("firm_contact_enter_activation", 0.90),
            0.90,
            allow_one=True,
        ),
        firm_contact_release_activation=_unit_interval_parameter(
            runtime.get("firm_contact_release_activation", 0.35),
            0.35,
            allow_one=True,
        ),
        activation_rise_sec=_nonnegative_float(
            runtime.get("activation_rise_sec", 0.10),
            "runtime.activation_rise_sec",
        ),
        activation_release_sec=_nonnegative_float(
            runtime.get("activation_release_sec", 0.12),
            "runtime.activation_release_sec",
        ),
    )


class FingertipContactStateMachine:
    """Latch one contact pair while its distance continuously controls takeover."""

    def __init__(self, config: FingertipContactConfig) -> None:
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._active_pair: str | None = None
        self._blend_pair: str | None = None
        self._candidate_pair: str | None = None
        self._candidate_since: float | None = None
        self._release_since: float | None = None
        self._activation = 0.0
        self._firm_contact = False
        self._last_time: float | None = None

    def update(self, ratios: Mapping[str, float], now_sec: float) -> FingertipContactDecision:
        now = float(now_sec)
        if not math.isfinite(now):
            raise ValueError("fingertip contact time must be finite")
        valid_ratios = {
            finger: float(value)
            for finger, value in ratios.items()
            if finger in self._config.profiles and math.isfinite(float(value)) and float(value) >= 0.0
        }
        event = self._update_latch(valid_ratios, now)
        self._update_activation(valid_ratios, now)
        return FingertipContactDecision(
            pair=self._blend_pair,
            activation=self._activation,
            ratios=valid_ratios,
            event=event,
        )

    def _update_latch(self, ratios: Mapping[str, float], now: float) -> str | None:
        if self._active_pair is not None:
            profile = self._config.profiles[self._active_pair]
            value = ratios.get(self._active_pair)
            if value is not None and self._takeover_target(profile, value) > 0.0:
                self._release_since = None
                return None
            if self._release_since is None:
                self._release_since = now
                return None
            if now - self._release_since >= self._config.release_hold_sec:
                released = self._active_pair
                self._active_pair = None
                self._candidate_pair = None
                self._candidate_since = None
                self._release_since = None
                return f"released:{released}"
            return None

        if self._blend_pair is not None and self._activation > 1e-6:
            return None

        candidate = self._candidate(ratios)
        if candidate is None:
            self._candidate_pair = None
            self._candidate_since = None
            return None
        if candidate != self._candidate_pair:
            self._candidate_pair = candidate
            self._candidate_since = now
            return None
        if self._candidate_since is None:
            self._candidate_since = now
            return None
        if now - self._candidate_since < self._config.min_hold_sec:
            return None

        self._active_pair = candidate
        self._blend_pair = candidate
        self._candidate_pair = None
        self._candidate_since = None
        return f"activated:{candidate}"

    def _candidate(self, ratios: Mapping[str, float]) -> str | None:
        candidates = sorted(
            (
                (float(value), finger)
                for finger, value in ratios.items()
                if self._takeover_target(self._config.profiles[finger], float(value)) > 0.0
            ),
            key=lambda item: item[0],
        )
        if not candidates:
            return None
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < self._config.candidate_gap_ratio:
            return None
        return candidates[0][1]

    def _takeover_target(self, profile: FingertipContactProfile, distance_ratio: float) -> float:
        contact_distance = profile.contact_distance_p95_ratio
        open_distance = profile.natural_open_distance_p05_ratio
        start_distance = open_distance - self._config.takeover_start_progress * (
            open_distance - contact_distance
        )
        full_distance = open_distance - self._config.full_takeover_progress * (
            open_distance - contact_distance
        )
        if distance_ratio >= start_distance:
            return 0.0
        if distance_ratio <= full_distance:
            return 1.0
        return (start_distance - distance_ratio) / (start_distance - full_distance)

    def _update_activation(self, ratios: Mapping[str, float], now: float) -> None:
        if self._last_time is None:
            self._last_time = now
            return
        elapsed = max(0.0, min(0.10, now - self._last_time))
        self._last_time = now
        if self._active_pair is not None:
            profile = self._config.profiles[self._active_pair]
            distance_ratio = ratios.get(self._active_pair)
            target = 0.0 if distance_ratio is None else self._takeover_target(profile, distance_ratio)
            target = self._firm_contact_target(target)
            duration = (
                self._config.activation_rise_sec
                if target >= self._activation
                else self._config.activation_release_sec
            )
            self._activation = _slew_towards(self._activation, target, elapsed, duration)
            return
        self._activation = _slew_towards(self._activation, 0.0, elapsed, self._config.activation_release_sec)
        if self._activation <= 1e-6:
            self._activation = 0.0
            self._blend_pair = None
            self._firm_contact = False

    def _firm_contact_target(self, target: float) -> float:
        enter = self._config.firm_contact_enter_activation
        release = min(enter, self._config.firm_contact_release_activation)
        if target >= enter:
            self._firm_contact = True
        elif target <= release:
            self._firm_contact = False
        return 1.0 if self._firm_contact else target


class FingertipContactCommandBlender:
    """Apply pair-local semantic contact timing to an already-retargeted L20 command."""

    def __init__(
        self,
        phase_config: FingertipContactPhaseConfig,
        open_command_for_slot: Callable[[int], int],
    ) -> None:
        self._phase_config = phase_config
        self._open_command_for_slot = open_command_for_slot
        self.reset()

    def reset(self) -> None:
        self._last_pair: str | None = None
        self._last_activation = 0.0
        self._is_closing = True

    def blend(
        self,
        command: list[int],
        profile: FingertipContactProfile,
        activation: float,
    ) -> list[int]:
        amount = max(0.0, min(1.0, float(activation)))
        if amount <= 0.0:
            self.reset()
            return list(command)
        slot_activations, slot_targets = self._phase_blend(profile, amount)
        return blend_fingertip_contact_command(
            command,
            profile,
            amount,
            slot_activations=slot_activations,
            slot_targets=slot_targets,
        )

    def _phase_blend(
        self,
        profile: FingertipContactProfile,
        activation: float,
    ) -> tuple[dict[int, float], dict[int, int]]:
        amount = max(0.0, min(1.0, float(activation)))
        pair = profile.finger
        last_activation = self._last_activation if pair == self._last_pair else 0.0
        deadband = self._phase_config.direction_change_deadband
        if amount > last_activation + deadband:
            self._is_closing = True
        elif amount < last_activation - deadband:
            self._is_closing = False

        orientation_slots = {5, 10}
        flexion_slots = [slot for slot in profile.override_slots if slot not in orientation_slots]
        slot_activations: dict[int, float] = {}
        slot_targets: dict[int, int] = {}
        phase = self._phase_config

        if self._is_closing:
            orientation_amount = _complete_early_progress(
                amount,
                phase.close_orientation_completion,
            )
            flexion_amount = _delayed_progress(
                amount,
                phase.close_flexion_start,
            )
            for slot in orientation_slots.intersection(profile.override_slots):
                slot_activations[slot] = orientation_amount
            for slot in flexion_slots:
                slot_activations[slot] = flexion_amount
        else:
            flexion_open_amount = _complete_early_progress(
                1.0 - amount,
                phase.release_flexion_open_completion,
            )
            orientation_amount = 1.0 - math.pow(
                1.0 - amount,
                phase.release_orientation_gamma,
            )
            for slot in orientation_slots.intersection(profile.override_slots):
                slot_activations[slot] = orientation_amount
            for slot in flexion_slots:
                slot_activations[slot] = flexion_open_amount
                slot_targets[slot] = clamp_u8(self._open_command_for_slot(slot))

        self._last_pair = pair
        self._last_activation = amount
        return slot_activations, slot_targets


def blend_fingertip_contact_command(
    command: list[int],
    profile: FingertipContactProfile,
    activation: float,
    *,
    slot_activations: Mapping[int, float] | None = None,
    slot_targets: Mapping[int, int] | None = None,
) -> list[int]:
    if len(command) != 20:
        raise ValueError("L20 contact blending needs a 20-slot command")
    amount = max(0.0, min(1.0, float(activation)))
    if amount <= 0.0 and not slot_activations:
        return list(command)

    output = [clamp_u8(value) for value in command]
    for slot in profile.override_slots:
        slot_amount = (
            max(0.0, min(1.0, float(slot_activations[slot])))
            if slot_activations is not None and slot in slot_activations
            else amount
        )
        if slot_amount <= 0.0:
            continue
        current = output[slot]
        target = (
            int(slot_targets[slot])
            if slot_targets is not None and slot in slot_targets
            else int(profile.contact_command[slot])
        )
        target_delta = target - current
        bounded_delta = max(-profile.max_command_delta, min(profile.max_command_delta, target_delta))
        output[slot] = clamp_u8(current + slot_amount * bounded_delta)
    return output


def _point(value: Any) -> np.ndarray:
    point = np.asarray(value, dtype=np.float64)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("fingertip contact point must be a finite 3-vector")
    return point


def _positive_float(value: Any, name: str) -> float:
    result = _nonnegative_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite nonnegative number") from exc
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be a finite nonnegative number")
    return result


def _finite_float(value: Any, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(result):
        return float(default)
    return result


def _unit_interval_float(value: Any, name: str) -> float:
    result = _nonnegative_float(value, name)
    if result >= 1.0:
        raise ValueError(f"{name} must be in [0.0, 1.0)")
    return result


def _unit_interval_parameter(value: Any, default: float, *, allow_one: bool) -> float:
    result = _finite_float(value, default)
    upper = 1.0 if allow_one else 1.0 - 1e-6
    return max(0.0, min(upper, result))


def _full_takeover_progress(value: Any, start_value: Any) -> float:
    start = _unit_interval_parameter(start_value, 0.35, allow_one=False)
    full = _unit_interval_parameter(value, 0.75, allow_one=True)
    return max(start + 1e-6, min(1.0, full))


def _nonnegative_int(value: Any, name: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a nonnegative integer") from exc
    if result < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return result


def _command(value: Any, name: str) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 20:
        raise ValueError(f"{name} must contain 20 L20 slot values")
    return tuple(clamp_u8(item) for item in value)


def _slots(value: Any, finger: str) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{finger}.override_slots must be a list")
    slots = tuple(int(item) for item in value)
    if len(set(slots)) != len(slots) or any(slot < 0 or slot >= 20 for slot in slots):
        raise ValueError(f"{finger}.override_slots must contain unique L20 slot indexes")
    if slots != FINGERTIP_CONTACT_SLOTS[finger]:
        raise ValueError(f"{finger}.override_slots must be {list(FINGERTIP_CONTACT_SLOTS[finger])}")
    return slots


def _slew_towards(current: float, target: float, elapsed: float, duration: float) -> float:
    if duration <= 1e-8:
        return target
    step = max(0.0, elapsed) / duration
    if target >= current:
        return min(target, current + step)
    return max(target, current - step)


def _complete_early_progress(value: float, completion: float) -> float:
    amount = max(0.0, min(1.0, float(value)))
    if completion <= 1e-8:
        return 1.0 if amount > 0.0 else 0.0
    return max(0.0, min(1.0, amount / completion))


def _delayed_progress(value: float, start: float) -> float:
    amount = max(0.0, min(1.0, float(value)))
    start = max(0.0, min(1.0 - 1e-6, float(start)))
    if amount <= start:
        return 0.0
    return max(0.0, min(1.0, (amount - start) / (1.0 - start)))
