#!/usr/bin/env bash
set -eo pipefail
gateway_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source '/home/wangjiaming/桌面/rm_nav_source_20261110(zbh)/scripts/local_env.sh'
# Test opens only self-created PTYs, never the C-board USB port.
exec python3 "$gateway_root/pc/source/rm_chassis_comm/test/test_ros_pty.py" \
  "$RM_NAV_RUNTIME/install/rm_chassis_comm/lib/rm_chassis_comm/chassis_node"
