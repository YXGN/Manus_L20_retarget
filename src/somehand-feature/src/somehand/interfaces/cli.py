"""Compatibility wrapper for the canonical CLI package."""

from somehand.cli import build_parser, main
from somehand.cli.commands import (
    _build_bihand_engine,
    _build_bihand_session,
    _build_engine,
    _build_runtime_session,
    _build_session,
    _finalize_bihand_run,
    _finalize_run,
    _print_bihand_startup,
    _print_startup,
    _run_bihand_dump_video,
    _run_bihand_hc_mocap_udp,
    _run_bihand_pico,
    _run_bihand_replay,
    _run_bihand_video,
    _run_bihand_webcam,
    _run_dump_video,
    _run_hc_mocap_udp,
    _run_pico,
    _run_replay,
    _run_video,
    _run_webcam,
    _wrap_bihand_source_for_interactive_recording,
    _wrap_bihand_source_for_recording,
    _wrap_source_for_interactive_recording,
    _wrap_source_for_recording,
)

__all__ = [name for name in globals() if name.startswith("_") or name in {"build_parser", "main"}]
