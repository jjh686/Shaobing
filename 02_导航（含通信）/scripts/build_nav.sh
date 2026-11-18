#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
[[ $# -le 1 ]] || { echo 'Usage: build_nav.sh [driver-install/setup.bash]' >&2; exit 2; }
if [[ $# -eq 1 ]]; then source "$1"; fi
ros2 pkg prefix livox_ros_driver2 >/dev/null
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cd "$RM_NAV_RUNTIME"
colcon --log-base "$RM_NAV_RUNTIME/log/build-nav" build --symlink-install \
  --executor sequential --base-paths "$RM_NAV_RUNTIME/source/src" \
  --build-base "$RM_NAV_RUNTIME/build" --install-base "$RM_NAV_RUNTIME/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  "-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=$RM_NAV_RUNTIME/deps/small_gicp"
