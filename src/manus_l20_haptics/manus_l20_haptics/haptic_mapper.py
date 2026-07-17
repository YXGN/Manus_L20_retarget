from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


FINGER_COUNT = 5


def clamp(value: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


def _five(values: Iterable[float], *, default: float = 0.0) -> list[float]:
    out = [float(value) for value in values]
    if len(out) < FINGER_COUNT:
        out.extend([default] * (FINGER_COUNT - len(out)))
    return out[:FINGER_COUNT]


@dataclass
class HapticMappingConfig:
    normal_force_full_scale: float = 100.0
    approach_full_scale: float = 200.0
    attack_alpha: float = 0.35
    release_alpha: float = 0.10
    contact_threshold: float = 0.05
    use_approach: bool = False
    max_intensity: float = 1.0
    invert_fingers: bool = False


class HapticMapper:
    """Map L20 force arrays to MANUS vibration intensities."""

    def __init__(self, config: HapticMappingConfig) -> None:
        self.config = config
        self._smoothed = [0.0] * FINGER_COUNT

    def reset(self) -> None:
        self._smoothed = [0.0] * FINGER_COUNT

    def update(
        self,
        normal_force: Iterable[float],
        approach_inc: Iterable[float] | None = None,
    ) -> list[float]:
        cfg = self.config
        normal = _five(normal_force)
        approach = _five(approach_inc or [])
        normal_full = max(float(cfg.normal_force_full_scale), 1e-6)
        approach_full = max(float(cfg.approach_full_scale), 1e-6)
        max_intensity = clamp(float(cfg.max_intensity))
        threshold = clamp(float(cfg.contact_threshold), 0.0, max_intensity)
        attack = clamp(float(cfg.attack_alpha))
        release = clamp(float(cfg.release_alpha))

        if cfg.invert_fingers:
            normal.reverse()
            approach.reverse()

        targets: list[float] = []
        for n, a in zip(normal, approach):
            raw = max(0.0, n) / normal_full
            if cfg.use_approach:
                raw = max(raw, max(0.0, a) / approach_full)
            raw = clamp(raw, 0.0, max_intensity)
            targets.append(0.0 if raw < threshold else raw)

        output: list[float] = []
        for index, target in enumerate(targets):
            current = self._smoothed[index]
            alpha = attack if target > current else release
            current += alpha * (target - current)
            current = clamp(current, 0.0, max_intensity)
            self._smoothed[index] = current
            output.append(current)
        return output
