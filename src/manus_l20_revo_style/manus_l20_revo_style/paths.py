from __future__ import annotations

import os
from pathlib import Path


def workspace_root() -> Path:
    env_root = os.environ.get("MANUS_L20_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / "src" / "manus_l20_retarget").exists():
            return parent
        if parent.name in {"install", "build"} and (parent.parent / "src" / "manus_l20_retarget").exists():
            return parent.parent
    raise RuntimeError("Cannot locate Manus_L20_retarget workspace root; set MANUS_L20_ROOT")


def resolve_workspace_path(path_value: str | Path) -> Path:
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (workspace_root() / path).resolve()
