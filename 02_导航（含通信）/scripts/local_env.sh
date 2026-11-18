#!/usr/bin/env bash
# Source from a launcher/build script. ROS Humble setup does not support set -u.
RM_NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RM_NAV_RUNTIME="/home/wangjiaming/.local/share/rm_sentry_nav_20261110"
export RM_NAV_ROOT RM_NAV_RUNTIME
python3 - "$RM_NAV_ROOT" "$RM_NAV_RUNTIME" <<'PY'
from pathlib import Path
import sys
source, runtime = map(Path, sys.argv[1:])
old = Path('/home/wangjiaming/桌面/pb2025_sentry_nav-main')
runtime.mkdir(parents=True, exist_ok=True)
links = {runtime/'source': source, runtime/'deps/small_gicp': old.parent/'small_gicp'}
for name in ('source', 'build', 'install'):
    links[Path('/tmp/pb2025_nav_'+name)] = old if name == 'source' else old/('runtime_'+name)
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
# Load only the proven MID-360S driver, not the old navigation packages.
source /tmp/pb2025_nav_install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash
if [[ -f "$RM_NAV_RUNTIME/install/local_setup.bash" ]]; then
    source "$RM_NAV_RUNTIME/install/local_setup.bash"
fi
export ROS_LOG_DIR="$RM_NAV_RUNTIME/log/ros"
mkdir -p -- "$ROS_LOG_DIR"
