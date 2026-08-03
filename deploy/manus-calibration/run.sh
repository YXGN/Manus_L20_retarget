#!/usr/bin/env bash
set -euo pipefail

APP_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SDK_LIB="$APP_DIR/lib/libManusSDK_Integrated.so"
WORKSPACE_SDK="$APP_DIR/../../src/ManusSDK/lib/libManusSDK_Integrated.so"
is_usable_sdk() {
    [[ -f "$1" ]] &&
        [[ "$(stat -c '%s' "$1" 2>/dev/null || echo 0)" -gt 1000000 ]] &&
        ! grep -q "git-lfs.github.com/spec" "$1" 2>/dev/null
}

if is_usable_sdk "$SDK_LIB"; then
    SDK_LIB_DIR="$(dirname "$SDK_LIB")"
elif is_usable_sdk "$WORKSPACE_SDK"; then
    # The workspace already contains the MANUS SDK used by manus_ros2. This
    # keeps the checked-in deployment pointer from blocking local use.
    SDK_LIB_DIR="$(dirname "$WORKSPACE_SDK")"
else
    cat >&2 <<EOF
Missing MANUS SDK runtime library:
  $SDK_LIB

The deployment copy is managed by Git LFS and should be about 117 MB. If this
is a source workspace, also check:

  git lfs install
  git lfs pull -I deploy/manus-calibration/lib/libManusSDK_Integrated.so

EOF
    exit 1
fi
export LD_LIBRARY_PATH="$SDK_LIB_DIR:$APP_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
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
