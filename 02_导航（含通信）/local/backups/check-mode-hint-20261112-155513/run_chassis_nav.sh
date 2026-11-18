#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
chassis_mode=${1:-check}
if [[ $# -gt 0 ]]; then shift; fi
chassis_connect=false
chassis_lidar=false
chassis_navigation=true
chassis_autostart=false
PORT=/dev/ttyACM0
chassis_map="$RM_NAV_RUNTIME/source/local/maps/20261110_210446/map.yaml"
chassis_pcd="$RM_NAV_RUNTIME/source/local/maps/20261110_210446/map.pcd"
case "$chassis_mode" in
  preview) chassis_navigation=false ;;
  check) ;;
  sensors) chassis_lidar=true ;;
  usb|navigation)
    [[ $# -gt 0 ]] || { echo '请填写云台 C 板的 /dev/serial/by-id/... 路径' >&2; exit 2; }
    PORT=$1; shift
    python3 /home/wangjiaming/桌面/c_board_nav_gateway_20261111/pc/check_port.py "$PORT" >/dev/null
    chassis_connect=true
    if [[ "$chassis_mode" == usb ]]; then
      chassis_navigation=false
    else
      chassis_lidar=true
      chassis_autostart=true
    fi ;;
  *) echo '用法: run_chassis_nav.sh preview | check | sensors | usb PORT | navigation PORT [launch参数]' >&2; exit 2 ;;
esac
exec ros2 launch rm_chassis_comm chassis_navigation.launch.py \
  "connect:=$chassis_connect" "serial_port:=$PORT" \
  "start_navigation:=$chassis_navigation" "start_lidar:=$chassis_lidar" \
  "autostart:=$chassis_autostart" \
  "mounting_config:=$RM_NAV_RUNTIME/source/local/measured_mounting.yaml" \
  "nav_params:=$RM_NAV_RUNTIME/source/local/chassis_nav2_params.yaml" \
  "rviz_config_file:=$RM_NAV_RUNTIME/source/local/chassis_nav.rviz" \
  "map:=$chassis_map" "prior_pcd_file:=$chassis_pcd" "$@"
