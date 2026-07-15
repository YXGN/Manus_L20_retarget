#!/usr/bin/env bash
set -euo pipefail

readonly SDK_PATH="src/ManusSDK/lib/libManusSDK_Integrated.so"
readonly SDK_OID="0e67141b97b64c089c3bbdab47980ca9822c4de19adea810a2f68722adcb3fe3"
readonly SDK_SIZE=117190696
readonly SDK_REF="7155c00ffaba90c37aa240539a1848ddca453edc"

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
git rev-parse --is-inside-work-tree >/dev/null

if [[ -f "$SDK_PATH" && "$(stat -c%s "$SDK_PATH")" == "$SDK_SIZE" ]]; then
  echo "MANUS Integrated SDK is already available: $SDK_PATH"
  exit 0
fi

command -v git-lfs >/dev/null || {
  echo "git-lfs is required; install it and rerun this script." >&2
  exit 1
}

git fetch --no-tags origin "$SDK_REF"
git lfs fetch origin "$SDK_REF" --include="$SDK_PATH"
mkdir -p "$(dirname "$SDK_PATH")"
object_path="$(git rev-parse --git-path "lfs/objects/${SDK_OID:0:2}/${SDK_OID:2:2}/$SDK_OID")"
install -m 0644 "$object_path" "$SDK_PATH"

[[ "$(stat -c%s "$SDK_PATH")" == "$SDK_SIZE" ]] || {
  echo "MANUS Integrated SDK download failed: $SDK_PATH" >&2
  exit 1
}
