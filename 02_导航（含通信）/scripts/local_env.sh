#!/usr/bin/env bash
# Source from a launcher/build script. ROS Humble setup does not support set -u.
RM_NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RM_NAV_RUNTIME="${RM_NAV_RUNTIME:-$HOME/.cache/rm_sentry_nav}"
export RM_NAV_ROOT RM_NAV_RUNTIME
python3 - "$RM_NAV_ROOT" "$RM_NAV_RUNTIME" <<'PY'
from pathlib import Path
import sys
source, runtime = map(Path, sys.argv[1:])
runtime.mkdir(parents=True, exist_ok=True)
links = {runtime/'source': source}
for link, target in links.items():
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.exists() or link.is_symlink():
        if link.resolve() != target.resolve():
            raise SystemExit('Workspace alias conflict: '+str(link))
    else:
        link.symlink_to(target, target_is_directory=True)
for name in ('build', 'install', 'log'):
    (runtime/name).mkdir(exist_ok=True)
PY
unset LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
RM_LIVOX_PREFIX="${RM_LIVOX_PREFIX:-$HOME/.cache/rm_lidar_link_test/install}"
if [[ -f "$RM_LIVOX_PREFIX/local_setup.bash" ]]; then
    source "$RM_LIVOX_PREFIX/local_setup.bash"
fi
case "$(uname -m)" in
    x86_64) rm_nav_sdk_arch=amd64 ;;
    aarch64) rm_nav_sdk_arch=arm64 ;;
    *) rm_nav_sdk_arch=unsupported ;;
esac
export LD_LIBRARY_PATH="$RM_NAV_ROOT/外部依赖源码/livox_ros_driver2/Livox-SDK2/lib/$rm_nav_sdk_arch${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ -f "$RM_NAV_RUNTIME/install/local_setup.bash" ]]; then
    source "$RM_NAV_RUNTIME/install/local_setup.bash"
fi
export ROS_LOG_DIR="$RM_NAV_RUNTIME/log/ros"
mkdir -p -- "$ROS_LOG_DIR"
