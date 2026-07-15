"""Canonical CLI package for l20_ik_core."""

from .main import main
from .parser import build_parser

__all__ = ["build_parser", "main"]
