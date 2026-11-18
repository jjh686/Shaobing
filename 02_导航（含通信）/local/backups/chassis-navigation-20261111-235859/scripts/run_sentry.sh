#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
MODE="${1:-preview}"
if [[ $# -gt 0 ]]; then shift; fi
CONNECT=false
NAVIGATION=false
LIDAR=true
AUTOSTART=true
PORT=/dev/ttyACM0
MAP="$RM_NAV_RUNTIME/source/local/maps/20261110_210446/map.yaml"
PCD="$RM_NAV_RUNTIME/source/local/maps/20261110_210446/map.pcd"
case "$MODE" in
  preview) ;;
  check) NAVIGATION=true; LIDAR=false; AUTOSTART=false ;;
  sensors) NAVIGATION=true; AUTOSTART=false ;;
  usb|navigation)
    [[ $# -gt 0 ]] || { echo 'Supply the gimbal C-board USB by-id path.' >&2; exit 2; }
    PORT="$1"; shift
    source "$RM_NAV_ROOT/scripts/validate_port.sh"
    CONNECT=true
    [[ "$MODE" != navigation ]] || NAVIGATION=true ;;
  *) echo 'Usage: run_sentry.sh preview | check | sensors | usb PORT | navigation PORT [launch arguments]' >&2; exit 2 ;;
esac
exec ros2 launch rm_chassis_comm sentry_reality.launch.py \
  "connect:=$CONNECT" "serial_port:=$PORT" "start_navigation:=$NAVIGATION" \
  "start_lidar:=$LIDAR" start_joystick:=false \
  "autostart:=$AUTOSTART" \
  "frame_config:=$RM_NAV_RUNTIME/source/local/sentry_real.yaml" \
  "nav_params:=$RM_NAV_RUNTIME/source/local/nav2_params.yaml" \
  "map:=$MAP" "prior_pcd_file:=$PCD" "$@"
