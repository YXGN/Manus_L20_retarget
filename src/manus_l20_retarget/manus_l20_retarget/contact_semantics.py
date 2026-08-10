from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable, Mapping

import numpy as np

from .mapping import clamp_u8


FINGERTIP_CONTACT_FINGERS = ("index",)
FINGERTIP_CONTACT_SLOTS = {
    # Contact takeover is local to the thumb and selected finger. Other fingers,
    # including their yaw slots, continue through normal teleoperation.
    "index": (0, 1, 5, 10, 15, 16),
}

CONTACT_STATE_IDLE = "idle"
CONTACT_STATE_ACTIVE = "active"
CONTACT_STATE_RELEASE_INTENT = "release_intent"
CONTACT_STATE_RECLOSE_INTENT = "reclose_intent"

# Backward-compatible names used by older tests/tools.
CONTACT_STATE_APPROACH = CONTACT_STATE_ACTIVE
CONTACT_STATE_HOLD = CONTACT_STATE_ACTIVE
CONTACT_STATE_RELEASE = CONTACT_STATE_RELEASE_INTENT
CONTACT_STATE_RECLOSE = CONTACT_STATE_RECLOSE_INTENT


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
    release_start_progress_delta: float
    reclose_start_progress_delta: float
    activation_rise_sec: float
    activation_release_sec: float
    distance_filter_alpha: float = 1.0
    command_slew_per_cycle: int = 255
    phase_switch_sec: float = 0.08


@dataclass(frozen=True, slots=True)
class FingertipContactFeature:
    finger: str
    distance_raw: float
    distance_filtered: float
    progress: float
    takeover_progress: float
    velocity: float


@dataclass(frozen=True, slots=True)
class FingertipContactDecision:
    pair: str | None
    activation: float
    ratios: dict[str, float]
    event: str | None
    state: str = CONTACT_STATE_IDLE
    phase: float = 0.0
    phase_target: float = 0.0
    features: dict[str, FingertipContactFeature] = field(default_factory=dict)
    slot_activations: dict[int, float] = field(default_factory=dict)
    slot_targets: dict[int, int] = field(default_factory=dict)
    base_command_slots: dict[int, int] = field(default_factory=dict)
    desired_command_slots: dict[int, int] = field(default_factory=dict)
    command_slots: dict[int, int] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class FingertipContactPhaseConfig:
    close_orientation_completion: float = 0.25
    close_flexion_start: float = 0.40
    release_flexion_open_completion: float = 0.65
    release_orientation_gamma: float = 2.5

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
            _unit_interval_parameter(self.release_flexion_open_completion, 0.65, allow_one=True),
        )
        object.__setattr__(
            self,
            "release_orientation_gamma",
            max(0.05, _finite_float(self.release_orientation_gamma, 2.5)),
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
        release_start_progress_delta=_unit_interval_parameter(
            runtime.get("release_start_progress_delta", 0.06),
            0.06,
            allow_one=True,
        ),
        reclose_start_progress_delta=_unit_interval_parameter(
            runtime.get("reclose_start_progress_delta", 0.02),
            0.02,
            allow_one=True,
        ),
        activation_rise_sec=_nonnegative_float(
            runtime.get("activation_rise_sec", 0.03),
            "runtime.activation_rise_sec",
        ),
        activation_release_sec=_nonnegative_float(
            runtime.get("activation_release_sec", 0.04),
            "runtime.activation_release_sec",
        ),
        distance_filter_alpha=_unit_interval_parameter(
            runtime.get("distance_filter_alpha", 1.0),
            1.0,
            allow_one=True,
        ),
        command_slew_per_cycle=_nonnegative_int(
            runtime.get("command_slew_per_cycle", 255),
            "runtime.command_slew_per_cycle",
        ),
        phase_switch_sec=_nonnegative_float(
            runtime.get("phase_switch_sec", 0.08),
            "runtime.phase_switch_sec",
        ),
    )


class FingertipContactFeatureExtractor:
    """Convert raw fingertip distance into progress and velocity."""

    def __init__(self, config: FingertipContactConfig) -> None:
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._last_filtered: dict[str, float] = {}
        self._last_time: float | None = None

    def update(self, ratios: Mapping[str, float], now_sec: float) -> dict[str, FingertipContactFeature]:
        now = float(now_sec)
        elapsed = 0.0 if self._last_time is None else max(1e-6, min(0.10, now - self._last_time))
        alpha = self._config.distance_filter_alpha
        features: dict[str, FingertipContactFeature] = {}
        for finger, value in ratios.items():
            if finger not in self._config.profiles:
                continue
            raw = float(value)
            if not math.isfinite(raw) or raw < 0.0:
                continue
            previous = self._last_filtered.get(finger)
            filtered = raw if previous is None else alpha * raw + (1.0 - alpha) * previous
            velocity = 0.0 if previous is None or elapsed <= 0.0 else (filtered - previous) / elapsed
            profile = self._config.profiles[finger]
            progress = _continuous_contact_progress(profile, filtered)
            features[finger] = FingertipContactFeature(
                finger=finger,
                distance_raw=raw,
                distance_filtered=filtered,
                progress=progress,
                takeover_progress=_takeover_contact_progress(self._config, progress),
                velocity=velocity,
            )
            self._last_filtered[finger] = filtered
        self._last_time = now
        return features


class ContactIntentStateMachine:
    """Decide contact intent and phase; it does not plan L20 slot values."""

    def __init__(self, config: FingertipContactConfig) -> None:
        self._config = config
        self.reset()

    @property
    def state(self) -> str:
        return self._state

    def reset(self) -> None:
        self._state = CONTACT_STATE_IDLE
        self._active_pair: str | None = None
        self._candidate_pair: str | None = None
        self._candidate_since: float | None = None
        self._release_since: float | None = None
        self._activation = 0.0
        self._phase = 0.0
        self._phase_target = 0.0
        self._peak_progress = 0.0
        self._trough_progress = 1.0
        self._reclose_complete_progress = 1.0
        self._last_time: float | None = None

    def update(
        self,
        features: Mapping[str, FingertipContactFeature],
        now_sec: float,
    ) -> FingertipContactDecision:
        now = float(now_sec)
        elapsed = 0.0 if self._last_time is None else max(0.0, min(0.10, now - self._last_time))
        self._last_time = now
        event, pair, progress = self._update_intent(features, now)
        self._update_activation(progress, elapsed)
        self._update_phase(elapsed)
        if self._active_pair is None and self._activation <= 1e-6:
            self._state = CONTACT_STATE_IDLE
            self._phase = 0.0
            self._phase_target = 0.0
        ratios = {finger: feature.distance_raw for finger, feature in features.items()}
        return FingertipContactDecision(
            pair=pair,
            activation=self._activation,
            ratios=ratios,
            event=event,
            state=self._state,
            phase=self._phase,
            phase_target=self._phase_target,
            features=dict(features),
        )

    def _update_intent(
        self,
        features: Mapping[str, FingertipContactFeature],
        now: float,
    ) -> tuple[str | None, str | None, float]:
        if self._active_pair is None:
            return self._update_idle(features, now)
        return self._update_active(features, now)

    def _update_idle(
        self,
        features: Mapping[str, FingertipContactFeature],
        now: float,
    ) -> tuple[str | None, str | None, float]:
        candidate = self._candidate(features)
        if candidate is None:
            self._candidate_pair = None
            self._candidate_since = None
            return None, None, 0.0
        if candidate != self._candidate_pair:
            self._candidate_pair = candidate
            self._candidate_since = now
            return None, None, 0.0
        if self._candidate_since is None:
            self._candidate_since = now
            return None, None, 0.0
        if now - self._candidate_since < self._config.min_hold_sec:
            return None, None, 0.0

        feature = features[candidate]
        self._active_pair = candidate
        self._state = CONTACT_STATE_ACTIVE
        self._phase_target = 0.0
        self._peak_progress = feature.progress
        self._trough_progress = feature.progress
        self._candidate_pair = None
        self._candidate_since = None
        return f"activated:{candidate}", candidate, feature.progress

    def _update_active(
        self,
        features: Mapping[str, FingertipContactFeature],
        now: float,
    ) -> tuple[str | None, str | None, float]:
        assert self._active_pair is not None
        feature = features.get(self._active_pair)
        if feature is None:
            return None, self._active_pair, 0.0

        progress = feature.progress
        event: str | None = None
        if self._state == CONTACT_STATE_ACTIVE:
            self._peak_progress = max(self._peak_progress, progress)
            self._trough_progress = progress
            if self._release_intent(feature):
                self._state = CONTACT_STATE_RELEASE_INTENT
                self._phase_target = _release_phase_target(progress)
                self._trough_progress = progress
                self._reclose_complete_progress = max(
                    self._config.full_takeover_progress,
                    min(1.0, self._peak_progress - self._config.release_start_progress_delta),
                )
        elif self._state == CONTACT_STATE_RELEASE_INTENT:
            self._trough_progress = min(self._trough_progress, progress)
            if self._reclose_intent(feature):
                self._state = CONTACT_STATE_RECLOSE_INTENT
                self._phase_target = _release_phase_target(progress)
            else:
                self._phase_target = _release_phase_target(progress)
        elif self._state == CONTACT_STATE_RECLOSE_INTENT:
            if self._release_intent(feature):
                self._state = CONTACT_STATE_RELEASE_INTENT
                self._phase_target = _release_phase_target(progress)
                self._trough_progress = progress
            elif progress >= self._reclose_complete_progress:
                self._state = CONTACT_STATE_ACTIVE
                self._phase_target = 0.0
                self._peak_progress = progress
                self._trough_progress = progress
            else:
                self._phase_target = _release_phase_target(progress)

        if progress <= 0.0:
            if self._release_since is None:
                self._release_since = now
            elif now - self._release_since >= self._config.release_hold_sec:
                released = self._active_pair
                self._active_pair = None
                self._candidate_pair = None
                self._candidate_since = None
                self._release_since = None
                self._peak_progress = 0.0
                self._trough_progress = 1.0
                self._phase_target = 0.0
                return f"released:{released}", None, 0.0
        else:
            self._release_since = None

        return event, self._active_pair, progress

    def _candidate(self, features: Mapping[str, FingertipContactFeature]) -> str | None:
        candidates = sorted(
            (
                (feature.distance_filtered, finger)
                for finger, feature in features.items()
                if feature.takeover_progress > 0.0
            ),
            key=lambda item: item[0],
        )
        if not candidates:
            return None
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < self._config.candidate_gap_ratio:
            return None
        return candidates[0][1]

    def _release_intent(self, feature: FingertipContactFeature) -> bool:
        return (
            feature.velocity > 0.0
            and self._peak_progress - feature.progress >= self._config.release_start_progress_delta
        )

    def _reclose_intent(self, feature: FingertipContactFeature) -> bool:
        return (
            feature.velocity < 0.0
            and feature.progress - self._trough_progress >= self._config.reclose_start_progress_delta
        )

    def _update_activation(self, target: float, elapsed: float) -> None:
        target = max(0.0, min(1.0, float(target)))
        duration = self._config.activation_rise_sec if target >= self._activation else self._config.activation_release_sec
        self._activation = _slew_towards(self._activation, target, elapsed, duration)
        if self._activation <= 1e-6 and target <= 0.0:
            self._activation = 0.0

    def _update_phase(self, elapsed: float) -> None:
        self._phase = _slew_towards(
            self._phase,
            self._phase_target,
            elapsed,
            self._config.phase_switch_sec,
        )


class FingertipContactStateMachine:
    """Compatibility wrapper around feature extraction and intent state."""

    def __init__(self, config: FingertipContactConfig) -> None:
        self._extractor = FingertipContactFeatureExtractor(config)
        self._intent = ContactIntentStateMachine(config)

    def reset(self) -> None:
        self._extractor.reset()
        self._intent.reset()

    def update(self, ratios: Mapping[str, float], now_sec: float) -> FingertipContactDecision:
        features = self._extractor.update(ratios, now_sec)
        return self._intent.update(features, now_sec)


class ContactSlotPlanner:
    """Map progress and release phase into smooth per-slot semantic commands."""

    ORIENTATION_SLOTS = {5, 10}

    def __init__(self, phase_config: FingertipContactPhaseConfig, open_command_for_slot: Callable[[int], int]) -> None:
        self._phase_config = phase_config
        self._open_command_for_slot = open_command_for_slot

    def plan(
        self,
        profile: FingertipContactProfile,
        progress: float,
        state: str,
        phase: float = 0.0,
    ) -> tuple[dict[int, float], dict[int, int]]:
        amount = max(0.0, min(1.0, float(progress)))
        release_phase = 1.0 if state == CONTACT_STATE_RELEASE_INTENT and phase <= 0.0 else phase
        return self._debug_plan(profile, amount, release_phase)

    def plan_command(
        self,
        command: list[int],
        profile: FingertipContactProfile,
        progress: float,
        phase: float,
    ) -> tuple[list[int], dict[int, float], dict[int, int]]:
        if len(command) != 20:
            raise ValueError("L20 contact planner needs a 20-slot command")
        amount = max(0.0, min(1.0, float(progress)))
        release_phase = max(0.0, min(1.0, float(phase)))
        close_gains, release_gains, release_targets = self._curves(profile, amount)
        output = [clamp_u8(value) for value in command]
        debug_gains: dict[int, float] = {}
        for slot in profile.override_slots:
            current = output[slot]
            contact = int(profile.contact_command[slot])
            close_value = _bounded_lerp_command(current, contact, close_gains[slot], profile.max_command_delta)
            release_target = release_targets.get(slot, contact)
            release_value = _bounded_lerp_command(current, release_target, release_gains[slot], profile.max_command_delta)
            output[slot] = clamp_u8(close_value + release_phase * (release_value - close_value))
            debug_gains[slot] = close_gains[slot] * (1.0 - release_phase) + release_gains[slot] * release_phase
        return output, debug_gains, release_targets

    def _debug_plan(
        self,
        profile: FingertipContactProfile,
        progress: float,
        phase: float,
    ) -> tuple[dict[int, float], dict[int, int]]:
        close_gains, release_gains, release_targets = self._curves(profile, progress)
        phase = max(0.0, min(1.0, float(phase)))
        gains = {
            slot: close_gains[slot] * (1.0 - phase) + release_gains[slot] * phase
            for slot in profile.override_slots
        }
        return gains, release_targets if phase > 0.0 else {}

    def _curves(
        self,
        profile: FingertipContactProfile,
        progress: float,
    ) -> tuple[dict[int, float], dict[int, float], dict[int, int]]:
        amount = max(0.0, min(1.0, float(progress)))
        close_orientation = _complete_early_progress(amount, self._phase_config.close_orientation_completion)
        close_flexion = _delayed_progress(amount, self._phase_config.close_flexion_start)
        release_flexion = _complete_early_progress(
            1.0 - amount,
            self._phase_config.release_flexion_open_completion,
        )
        release_orientation = 1.0 - math.pow(1.0 - amount, self._phase_config.release_orientation_gamma)
        close_gains: dict[int, float] = {}
        release_gains: dict[int, float] = {}
        release_targets: dict[int, int] = {}
        for slot in profile.override_slots:
            if slot in self.ORIENTATION_SLOTS:
                close_gains[slot] = close_orientation
                release_gains[slot] = release_orientation
            else:
                close_gains[slot] = close_flexion
                release_gains[slot] = release_flexion
                release_targets[slot] = clamp_u8(self._open_command_for_slot(slot))
        return close_gains, release_gains, release_targets


class ContactCommandSmoother:
    """Optional per-slot output slew after semantic planning."""

    def __init__(self, max_step_per_cycle: int) -> None:
        self._max_step = max(0, int(max_step_per_cycle))
        self.reset()

    def reset(self) -> None:
        self._last_output: dict[int, int] = {}

    def apply(
        self,
        base_command: list[int],
        desired_command: list[int],
        slots: tuple[int, ...],
    ) -> list[int]:
        if self._max_step >= 255:
            output = list(desired_command)
            for slot in slots:
                self._last_output[slot] = clamp_u8(output[slot])
            return output

        output = list(desired_command)
        for slot in slots:
            desired = clamp_u8(desired_command[slot])
            previous = self._last_output.get(slot, clamp_u8(base_command[slot]))
            if self._max_step <= 0:
                value = previous
            else:
                value = _step_u8_towards(previous, desired, self._max_step)
            output[slot] = value
            self._last_output[slot] = value
        return output


class FingertipContactCommandBlender:
    """Apply slot-phase planning to an already-retargeted L20 command."""

    def __init__(
        self,
        phase_config: FingertipContactPhaseConfig,
        open_command_for_slot: Callable[[int], int],
        *,
        command_slew_per_cycle: int = 255,
    ) -> None:
        self._planner = ContactSlotPlanner(phase_config, open_command_for_slot)
        self._smoother = ContactCommandSmoother(command_slew_per_cycle)
        self.reset()

    def reset(self) -> None:
        self._smoother.reset()

    def blend(
        self,
        command: list[int],
        profile: FingertipContactProfile,
        activation: float,
        *,
        state: str = CONTACT_STATE_ACTIVE,
    ) -> list[int]:
        phase = 1.0 if state == CONTACT_STATE_RELEASE_INTENT else 0.0
        desired, _, _ = self._planner.plan_command(command, profile, activation, phase)
        return self._smoother.apply(command, desired, profile.override_slots)


class FingertipContactController:
    """Single entry point for semantic fingertip contact retargeting."""

    def __init__(
        self,
        config: FingertipContactConfig,
        phase_config: FingertipContactPhaseConfig,
        open_command_for_slot: Callable[[int], int],
    ) -> None:
        self._config = config
        self._extractor = FingertipContactFeatureExtractor(config)
        self._intent = ContactIntentStateMachine(config)
        self._planner = ContactSlotPlanner(phase_config, open_command_for_slot)
        self._smoother = ContactCommandSmoother(config.command_slew_per_cycle)

    def reset(self) -> None:
        self._extractor.reset()
        self._intent.reset()
        self._smoother.reset()

    def apply(self, command: list[int], skeleton: Any, now_sec: float) -> tuple[list[int], FingertipContactDecision]:
        ratios = thumb_fingertip_distance_ratios(skeleton)
        features = self._extractor.update(ratios, now_sec)
        decision = self._intent.update(features, now_sec)
        if decision.pair is None or decision.activation <= 0.0:
            self._smoother.reset()
            return list(command), decision

        profile = self._config.profiles[decision.pair]
        base_slots = {slot: clamp_u8(command[slot]) for slot in profile.override_slots}
        desired, slot_activations, slot_targets = self._planner.plan_command(
            command,
            profile,
            decision.activation,
            decision.phase,
        )
        output = self._smoother.apply(command, desired, profile.override_slots)
        decision = FingertipContactDecision(
            pair=decision.pair,
            activation=decision.activation,
            ratios=decision.ratios,
            event=decision.event,
            state=decision.state,
            phase=decision.phase,
            phase_target=decision.phase_target,
            features=decision.features,
            slot_activations=slot_activations,
            slot_targets=slot_targets,
            base_command_slots=base_slots,
            desired_command_slots={slot: desired[slot] for slot in profile.override_slots},
            command_slots={slot: output[slot] for slot in profile.override_slots},
        )
        return output, decision


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
        output[slot] = _bounded_lerp_command(current, target, slot_amount, profile.max_command_delta)
    return output


def _profile_span(profile: FingertipContactProfile) -> float:
    return max(1e-8, profile.natural_open_distance_p05_ratio - profile.contact_distance_p95_ratio)


def _continuous_contact_progress(profile: FingertipContactProfile, distance_ratio: float) -> float:
    return max(
        0.0,
        min(
            1.0,
            (profile.natural_open_distance_p05_ratio - distance_ratio) / _profile_span(profile),
        ),
    )


def _takeover_contact_progress(config: FingertipContactConfig, progress: float) -> float:
    amount = max(0.0, min(1.0, float(progress)))
    if amount <= config.takeover_start_progress:
        return 0.0
    if amount >= config.full_takeover_progress:
        return 1.0
    return (amount - config.takeover_start_progress) / (
        config.full_takeover_progress - config.takeover_start_progress
    )


def _release_phase_target(progress: float) -> float:
    return max(0.0, min(1.0, 1.0 - float(progress)))


def _bounded_lerp_command(current: int, target: int, amount: float, max_delta: int) -> int:
    current = clamp_u8(current)
    target = clamp_u8(target)
    bounded_delta = max(-int(max_delta), min(int(max_delta), target - current))
    return clamp_u8(current + max(0.0, min(1.0, float(amount))) * bounded_delta)


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


def _step_u8_towards(current: int, target: int, max_step: int) -> int:
    current = clamp_u8(current)
    target = clamp_u8(target)
    max_step = max(0, int(max_step))
    if abs(target - current) <= max_step:
        return target
    return clamp_u8(current + max_step if target > current else current - max_step)


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
