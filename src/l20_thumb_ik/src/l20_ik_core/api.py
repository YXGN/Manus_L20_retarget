"""Stable public API for embedding l20_ik_core as a retargeting library."""

from l20_ik_core.application import BiHandRetargetingEngine, RetargetingEngine
from l20_ik_core.domain import (
    BiHandFrame,
    BiHandRetargetingConfig,
    BiHandRetargetingResult,
    HandFrame,
    RetargetingConfig,
    RetargetingStepResult,
)
from l20_ik_core.infrastructure.config_loader import load_bihand_config, load_retargeting_config

__all__ = [
    "BiHandFrame",
    "BiHandRetargetingConfig",
    "BiHandRetargetingEngine",
    "BiHandRetargetingResult",
    "HandFrame",
    "RetargetingConfig",
    "RetargetingEngine",
    "RetargetingStepResult",
    "load_bihand_config",
    "load_retargeting_config",
]
