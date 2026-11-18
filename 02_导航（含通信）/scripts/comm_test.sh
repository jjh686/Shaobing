#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
ros2 pkg prefix rm_chassis_comm >/dev/null
MODE="${1:-preview}"
CONNECT=false
PORT=/dev/ttyACM0
case "$MODE" in
  preview) [[ $# -le 1 ]] || exit 2 ;;
  usb)
    [[ $# -eq 2 ]] || { echo 'Usage: comm_test.sh usb /dev/serial/by-id/DEVICE' >&2; exit 2; }
    PORT="$2"
    source "$RM_NAV_ROOT/scripts/validate_port.sh"
    CONNECT=true ;;
  *) echo 'Usage: comm_test.sh preview | usb /dev/serial/by-id/DEVICE' >&2; exit 2 ;;
esac
exec ros2 run rm_chassis_comm chassis_node --ros-args \
  --params-file "$RM_NAV_RUNTIME/source/src/rm_chassis_comm/config/chassis.yaml" \
  -r __ns:=/chassis -r cmd_vel:=/chassis/test_unused_cmd_vel \
  -r cmd_vel_stamped:=/chassis/test_unused_cmd_vel_stamped \
  -p route:=2 -p connect:="$CONNECT" -p serial_port:="$PORT" \
  -p enabled:=false -p sp_proxy_enabled:=false -p use_stamped_commands:=true
