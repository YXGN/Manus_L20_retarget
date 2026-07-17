from __future__ import annotations

import argparse
import json

from .core import RevoStyleL20Retarget, load_yaml
from .paths import resolve_workspace_path


def _sample_ergonomics(close_amount: float) -> dict[str, float]:
    close = max(0.0, min(1.0, close_amount))
    spread = 10.0 * (1.0 - close)
    values: dict[str, float] = {
        "IndexMCPStretch": 85.0 * close,
        "IndexPIPStretch": 75.0 * close,
        "IndexDIPStretch": 55.0 * close,
        "MiddleMCPStretch": 80.0 * close,
        "MiddlePIPStretch": 90.0 * close,
        "MiddleDIPStretch": 65.0 * close,
        "RingMCPStretch": 75.0 * close,
        "RingPIPStretch": 90.0 * close,
        "RingDIPStretch": 60.0 * close,
        "PinkyMCPStretch": 60.0 * close,
        "PinkyPIPStretch": 80.0 * close,
        "PinkyDIPStretch": 45.0 * close,
        "ThumbMCPStretch": 35.0 * close,
        "ThumbPIPStretch": 25.0 * close,
        "ThumbDIPStretch": 25.0 * close,
        "ThumbMCPSpread": 20.0 * close,
        "IndexSpread": spread,
        "MiddleSpread": 0.0,
        "RingSpread": -spread,
        "PinkySpread": -1.5 * spread,
    }
    return values


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline Revo-style MANUS -> L20 command demo.")
    parser.add_argument(
        "--config",
        default="src/manus_l20_revo_style/config/revo_style_l20_right.yaml",
        help="Config YAML path, absolute or relative to MANUS_L20_ROOT.",
    )
    parser.add_argument("--close", type=float, default=0.75, help="Synthetic close amount in [0, 1].")
    args = parser.parse_args()

    config_path = resolve_workspace_path(args.config)
    retarget = RevoStyleL20Retarget(load_yaml(config_path))
    targets, command = retarget.retarget(_sample_ergonomics(args.close), smooth=False)
    print(
        json.dumps(
            {
                "config": str(config_path),
                "command": command,
                "target_preview_deg": {
                    key: round(value * 180.0 / 3.141592653589793, 3)
                    for key, value in list(targets.items())[:10]
                },
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
