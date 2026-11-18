#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cd "$RM_NAV_RUNTIME"
colcon --log-base "$RM_NAV_RUNTIME/log/build-comm" build --symlink-install \
  --executor sequential --base-paths "$RM_NAV_RUNTIME/source/src/rm_chassis_comm" \
  --build-base "$RM_NAV_RUNTIME/build" --install-base "$RM_NAV_RUNTIME/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING="${RM_BUILD_TESTING:-ON}"
