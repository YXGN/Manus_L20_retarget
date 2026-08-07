#!/usr/bin/env bash
set -eo pipefail

sanitize_library_path() {
  local cleaned_library_path=""
  local library_dir
  local library_dirs=()
  IFS=':' read -r -a library_dirs <<< "${LD_LIBRARY_PATH:-}"
  for library_dir in "${library_dirs[@]}"; do
    if [[ -n "$library_dir" && "$library_dir" != /snap/* ]]; then
      cleaned_library_path="${cleaned_library_path:+$cleaned_library_path:}$library_dir"
    fi
  done
  export LD_LIBRARY_PATH="$cleaned_library_path"
}

sanitize_python_path() {
  local cleaned_path=""
  local path_dir
  local path_dirs=()
  IFS=':' read -r -a path_dirs <<< "${PATH:-}"
  for path_dir in "${path_dirs[@]}"; do
    if [[ -n "$path_dir" &&
      "$path_dir" != /opt/miniconda3 &&
      "$path_dir" != /opt/miniconda3/* &&
      ( -z "${CONDA_PREFIX:-}" || ( "$path_dir" != "$CONDA_PREFIX" && "$path_dir" != "$CONDA_PREFIX"/* ) ) ]]; then
      cleaned_path="${cleaned_path:+$cleaned_path:}$path_dir"
    fi
  done
  export PATH="$cleaned_path"
}

sanitize_library_path
sanitize_python_path
package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$package_root"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -x "$workspace_root/install/manus_ros2/lib/manus_ros2/manus_data_publisher" && -f "$workspace_root/install/setup.bash" ]]; then
  source "$workspace_root/install/setup.bash"
else
  outer_workspace="$(cd "$package_root/../.." && pwd)"
  if [[ -f "$outer_workspace/install/share/manus_l20_retarget/package.xml" && -f "$outer_workspace/install/setup.bash" ]]; then
    source "$outer_workspace/install/setup.bash"
  fi
fi

# Conda/Snap shells can leak an incompatible Core20 glibc into system Python.
sanitize_library_path
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export PYTHONPATH="$package_root/src/manus_l20_retarget${PYTHONPATH:+:$PYTHONPATH}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog-manus-calibration-ui}"

publisher_source_changed() {
  local source_file
  local installed_publisher="$package_root/install/manus_ros2/lib/manus_ros2/manus_data_publisher"
  [[ ! -x "$installed_publisher" ]] && return 0
  for source_file in \
    "$package_root/src/manus_ros2/CMakeLists.txt" \
    "$package_root/src/manus_ros2/src/ManusDataPublisher.cpp" \
    "$package_root/src/manus_ros2/src/ManusDataPublisher.hpp"; do
    [[ "$source_file" -nt "$installed_publisher" ]] && return 0
  done
  return 1
}

if publisher_source_changed; then
  if [[ ! -f "$package_root/src/ManusSDK/lib/libManusSDK_Integrated.so" ]]; then
    echo "缺少 MANUS Integrated SDK 库：src/ManusSDK/lib/libManusSDK_Integrated.so" >&2
    echo "请先放入该文件；如果这是完整 git 仓库，可运行：scripts/fetch_manus_sdk.sh" >&2
    exit 1
  fi
  colcon_build_args=(--symlink-install --packages-select manus_ros2_msgs manus_ros2)
  if [[ "$workspace_root" != "${workspace_root//[![:ascii:]]/}" ]]; then
    colcon_build_args+=(
      --build-base "${MANUS_COLCON_BUILD_BASE:-$HOME/.cache/manus_l20_retarget/build}"
      --install-base "$workspace_root/install"
    )
  fi
  echo "正在增量编译 MANUS Qt 标定后端…"
  colcon build "${colcon_build_args[@]}" --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
  source "$package_root/install/setup.bash"
fi

manus_publisher_pid=""
publisher_is_running() {
  pgrep -f '/manus_ros2/manus_data_publisher([[:space:]]|$)' >/dev/null 2>&1 ||
    timeout 2s ros2 node list --no-daemon 2>/dev/null | grep -Eq '(^|/)manus_data_publisher$'
}

stop_manus_publisher() {
  if [[ -n "$manus_publisher_pid" ]] && kill -0 "$manus_publisher_pid" 2>/dev/null; then
    kill -INT "$manus_publisher_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$manus_publisher_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$manus_publisher_pid" 2>/dev/null; then
      kill -TERM "$manus_publisher_pid" 2>/dev/null || true
    fi
    wait "$manus_publisher_pid" 2>/dev/null || true
  fi
}

trap stop_manus_publisher EXIT

if ! publisher_is_running; then
  publisher_prefix="$(ros2 pkg prefix manus_ros2 2>/dev/null)" || {
    echo "找不到 manus_ros2，请先编译并 source 工作区。" >&2
    exit 1
  }
  publisher_executable="$publisher_prefix/lib/manus_ros2/manus_data_publisher"
  if [[ ! -x "$publisher_executable" ]]; then
    echo "找不到 MANUS 发布器：$publisher_executable" >&2
    exit 1
  fi
  echo "正在自动启动 MANUS 数据发布器…"
  "$publisher_executable" --ros-args \
    -p "calibration_output_dir:=$package_root/src/manus_ros2/calibration" &
  manus_publisher_pid=$!
  sleep 0.5
  if ! kill -0 "$manus_publisher_pid" 2>/dev/null; then
    wait "$manus_publisher_pid" || true
    manus_publisher_pid=""
    echo "MANUS 数据发布器启动失败，请检查 SDK、Dongle 和 USB 权限。" >&2
    exit 1
  fi
fi

if ! timeout 3s ros2 topic list --no-daemon 2>/dev/null | grep -Fxq '/manus/calibration/command'; then
  echo "当前 manus_data_publisher 不包含 Qt 标定接口；请先停止旧发布器后重新运行。" >&2
  exit 1
fi

set -u
/usr/bin/python3 "$package_root/tools/manus_l20_calibration_ui.py" "$@"
