#!/usr/bin/env bash
set -euo pipefail

APP_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export LD_LIBRARY_PATH="$APP_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ -z "${MANUS_CALIBRATION_DIR:-}" ]]; then
    probe_dir=$APP_DIR
    while [[ "$probe_dir" != "/" ]]; do
        candidate="$probe_dir/src/manus_ros2/calibration"
        if [[ -d "$candidate" ]]; then
            export MANUS_CALIBRATION_DIR="$candidate"
            break
        fi
        probe_dir=$(dirname "$probe_dir")
    done
fi
exec "$APP_DIR/bin/CalibrationGUI.out" "$@"
